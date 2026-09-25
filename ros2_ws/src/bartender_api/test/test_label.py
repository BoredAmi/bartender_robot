"""Label reading -> inventory row, from what PaddleOCR and Gemini read in phone photos."""
import os
import sys
import threading
import time

import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir))

from bartender_api.drink import label                       # noqa: E402

INVENTORY = label.load_inventory(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir, 'config',
    'bottles.yaml'))

# PaddleOCR on bottle crops from phone photos, at ocr.SCALE; Żubrówka's label read as noise.
OCR_TEXT = {
    'jd_949': "MF IOK DSWIELS JACKDANIEL'S 26N Tennessee WHISKEY Jara 40% VoL. 70c",
    'jd_952': "MF PACK DANIES JACK DANIEL'S 207 BRAND Tennessee WHISKEY 40% VhL. 70CL",
    'zub_949': '20 BRO MF 鞋 AJA18 50',
    'gin_949': '天雀 GIN',
    'gin_952': '5 GIN',
}


def _read(brand='', type_='unknown', volume_ml=0, text_read=''):
    return {'brand': brand, 'type': type_, 'volume_ml': volume_ml,
            'abv_percent': 40.0, 'text_read': text_read}


def _ocr(text):
    return lambda crop: (text, None)


def _gemini(answer):
    asked = []

    def ask(crop):
        asked.append(crop)
        return answer
    return ask, asked


@pytest.mark.parametrize('text, expected', [
    (OCR_TEXT['jd_949'], ('whiskey', 700, 40.0)),
    (OCR_TEXT['jd_952'], ('whiskey', 700, 40.0)),
    (OCR_TEXT['zub_949'], ('unknown', 0, 0.0)),
    (OCR_TEXT['gin_952'], ('gin', 0, 0.0)),
    ('BIAŁA POLSKA WÓDKA 40% vol 500 ml', ('vodka', 500, 40.0)),
    ('Tequila 38 % alc 0,75 l', ('tequila', 750, 38.0)),
    ('4%0 GINGER', ('unknown', 0, 0.0)),
])
def test_parse_pulls_type_volume_and_abv_from_ocr_text(text, expected):
    got = label.parse(text)
    assert (got['type'], got['volume_ml'], got['abv_percent']) == expected


@pytest.mark.parametrize('text, brand', [
    (OCR_TEXT['jd_949'], "Jack Daniel's"),
    (OCR_TEXT['jd_952'], "Jack Daniel's"),
    (OCR_TEXT['gin_949'], 'Tenjaku'),
])
def test_ocr_naming_a_stocked_brand_wins_over_gemini(text, brand):
    ask, _ = _gemini((_read('Patron', 'tequila', 750, 'PATRON TEQUILA'), None))
    got = label.read_label('crop', INVENTORY, _ocr(text), ask)
    assert (got.brand, got.source, got.confidence) == (brand, 'inventory', 1.0)


def test_ocr_and_gemini_run_at_the_same_time():
    gemini_started = threading.Event()

    def ocr(crop):
        # Only returns if Gemini was started while OCR was still reading.
        assert gemini_started.wait(5), 'Gemini did not start alongside OCR'
        return OCR_TEXT['gin_952'], None

    def ask(crop):
        gemini_started.set()
        return _read("Hendrick's", 'gin', 700, "HENDRICK'S GIN"), None
    assert label.read_label('crop', INVENTORY, ocr, ask).brand == "Hendrick's"


def test_ocr_answer_does_not_wait_for_a_slow_gemini():
    release = threading.Event()

    def slow_ask(crop):
        release.wait(5)
        return None, 'gemini: late'
    start = time.monotonic()
    got = label.read_label('crop', INVENTORY, _ocr(OCR_TEXT['jd_949']), slow_ask)
    elapsed = time.monotonic() - start
    release.set()
    assert got.brand == "Jack Daniel's"
    assert elapsed < 1.0


def test_ocr_type_only_asks_gemini_and_gemini_wins():
    ask, asked = _gemini((_read("Hendrick's", 'gin', 700, "HENDRICK'S GIN"), None))
    got = label.read_label('crop', INVENTORY, _ocr(OCR_TEXT['gin_952']), ask)
    assert (got.brand, got.source) == ("Hendrick's", 'gemini')
    assert asked == ['crop']


def test_gemini_down_keeps_ocrs_partial_answer():
    ask, _ = _gemini((None, 'gemini: 429 quota'))
    got = label.read_label('crop', INVENTORY, _ocr(OCR_TEXT['gin_952']), ask)
    assert (got.brand, got.confidence) == ('Tenjaku', label.TYPE_ONLY_CONFIDENCE)


def test_offline_new_bottle_gets_its_type_but_no_brand():
    got = label.read_label('crop', INVENTORY, _ocr('BACARDI CARTA BLANCA RUM 70cl'))
    assert (got.brand, got.type, got.volume_ml) == (None, 'rum', 700)
    assert (got.source, got.confidence) == ('ocr', label.TYPE_READ_CONFIDENCE)


def test_nothing_read_anywhere_is_unknown_with_both_reasons():
    ask, _ = _gemini((None, 'gemini: 503'))
    got = label.read_label('crop', INVENTORY, _ocr(OCR_TEXT['zub_949']), ask)
    assert got.type is None
    assert 'ocr read no stocked brand' in got.reason
    assert 'gemini: 503' in got.reason


def test_gemini_alone_works_without_ocr():
    ask, _ = _gemini((_read('ŻUBRÓWKA', 'vodka', 0, 'ŻUBRÓWKA VODKA BISON GRASS'), None))
    got = label.read_label('crop', INVENTORY, None, ask)
    assert (got.brand, got.volume_ml) == ('Żubrówka', 500)


def test_ocr_failure_is_reported():
    got = label.read_label('crop', INVENTORY, lambda crop: (None, 'ocr: boom'))
    assert got.type is None
    assert got.reason == 'ocr: boom'


def test_no_reader_at_all_says_so():
    assert 'no OCR' in label.read_label('crop', INVENTORY).reason


def test_accents_and_case_do_not_matter():
    got = label.match(_read('ZUBROWKA', 'vodka', 500, 'BIALA POLSKA WODKA'),
                      INVENTORY, 'gemini')
    assert got.brand == 'Żubrówka'


def test_type_and_volume_come_from_the_inventory_not_the_reading():
    got = label.match(_read("Jack Daniel's", 'other', 1000), INVENTORY, 'gemini')
    assert (got.type, got.volume_ml) == ('whiskey', 700)
    assert got.read['volume_ml'] == 1000


def test_volume_read_breaks_a_tie_between_sizes_of_one_brand():
    stock = INVENTORY + [label.Bottle("Jack Daniel's", 'whiskey', 1000)]
    assert label.match(_read("Jack Daniel's", 'whiskey', 1000), stock,
                       'gemini').volume_ml == 1000
    assert label.match(_read("Jack Daniel's", 'whiskey', 700), stock,
                       'gemini').volume_ml == 700


def test_new_bottle_is_reported_as_gemini_read_it():
    got = label.match(_read("Hendrick's", 'gin', 700, "HENDRICK'S GIN 41.4% 70cl"),
                      INVENTORY, 'gemini')
    assert (got.brand, got.type, got.volume_ml) == ("Hendrick's", 'gin', 700)
    assert (got.source, got.confidence) == ('gemini', label.READ_CONFIDENCE)


def test_brand_or_type_absent_from_the_text_is_a_guess_and_not_reported():
    # From above, Gemini named the Jack Daniel's bottle Patron with no label text.
    got = label.match(_read('Patron', 'tequila', 750, ''), INVENTORY, 'gemini')
    assert got.type is None
    assert got.read['brand'] == 'Patron'


def test_type_spelled_on_the_label_of_an_ambiguous_type_has_no_brand():
    stock = INVENTORY + [label.Bottle('Absolut', 'vodka', 700)]
    got = label.match(_read('', 'vodka', 0, 'VODKA 40%'), stock, 'gemini')
    assert (got.brand, got.type) == (None, 'vodka')


def test_empty_inventory_still_reports_what_gemini_read():
    got = label.match(_read("Jack Daniel's", 'whiskey', 700, "JACK DANIEL'S"), [],
                      'gemini')
    assert (got.brand, got.source) == ("Jack Daniel's", 'gemini')


def test_schema_offers_general_and_inventory_types():
    stock = INVENTORY + [label.Bottle('Soju Co', 'soju', 360)]
    enum = label.schema(stock)['properties']['type']['enum']
    assert {'rum', 'tequila', 'whiskey', 'soju'} <= set(enum)
    assert enum[-2:] == ['other', 'unknown']
