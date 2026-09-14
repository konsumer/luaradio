import math
import numpy
from generate import *


def hex_to_bits(hexstr):
    return numpy.array([int(c, 16) >> s & 1 for c in hexstr for s in (3, 2, 1, 0)], dtype=numpy.bool_)


def frame_vector(frames):
    """Build an expected vector of frames. Frame fields are optional, so each
    frame is passed to the ADSBFrameType constructor as a single table."""
    return "require('radio.blocks.protocol.adsbframer').ADSBFrameType.vector_from_array(%s)" % serialize([[f] for f in frames])


def lua_string(s):
    # The generator serializes unknown types with str(), so Lua string
    # literals have to be quoted here
    return '"%s"' % s


def generate():
    vectors = []

    identification = hex_to_bits("8D4840D6202CC371C32CE0576098")
    vectors.append(TestVector([], [identification], [frame_vector([
        {"df": 17, "icao": 0x4840D6, "hex": lua_string("8d4840d6202cc371c32ce0576098"),
         "type_code": 4, "category": 0, "callsign": lua_string("KLM1023")}])],
        "Aircraft identification"))

    # An airborne position report is a compact position report of a grid of
    # zones, decoded to an absolute position only when both an even and an odd
    # report have been received
    position_even = hex_to_bits("8D40621D58C382D690C8AC2863A7")
    position_odd = hex_to_bits("8D40621D58C386435CC412692AD6")
    even_frame = {"df": 17, "icao": 0x40621D, "hex": lua_string("8d40621d58c382d690c8ac2863a7"),
                  "type_code": 11, "altitude": 38000, "surveillance_status": 0,
                  "cpr_format": 0, "cpr_lat": 93000, "cpr_lon": 51372}
    odd_frame = {"df": 17, "icao": 0x40621D, "hex": lua_string("8d40621d58c386435cc412692ad6"),
                 "type_code": 11, "altitude": 38000, "surveillance_status": 0,
                 "cpr_format": 1, "cpr_lat": 74158, "cpr_lon": 50194}

    vectors.append(TestVector([], [position_even], [frame_vector([even_frame])],
        "Airborne position, single even report"))

    vectors.append(TestVector([], [numpy.hstack((position_even, position_odd))], [frame_vector([
        even_frame,
        dict(odd_frame, latitude=52.26578017412606, longitude=3.938912527901786)])],
        "Airborne position, even and odd report pair"))

    # Surface position reports only encode the low order bits of a 90 degree
    # zone, so a receiver reference position is required
    surface_even = hex_to_bits("8C4841753AAB238733C8CD4020B1")
    surface_odd = hex_to_bits("8C4841753A8A35323FAEBDAC702D")
    surface_options = {"reference": {"lat": 51.990, "lon": 4.375}}
    surface_even_frame = {"df": 17, "icao": 0x484175, "hex": lua_string("8c4841753aab238733c8cd4020b1"),
                          "type_code": 7, "groundspeed": 18.0, "track": 140.625,
                          "cpr_format": 0, "cpr_lat": 115609, "cpr_lon": 116941,
                          "latitude": 52.32304000854492, "longitude": 4.730472564697266}
    surface_odd_frame = {"df": 17, "icao": 0x484175, "hex": lua_string("8c4841753a8a35323faebdac702d"),
                         "type_code": 7, "groundspeed": 16.0, "track": 98.4375,
                         "cpr_format": 1, "cpr_lat": 39199, "cpr_lon": 110269,
                         "latitude": 52.320607072215964, "longitude": 4.734734671456465}

    vectors.append(TestVector([surface_options], [numpy.hstack((surface_even, surface_odd))], [frame_vector([
        surface_even_frame, surface_odd_frame])],
        "Surface position, even and odd report pair"))

    # Airborne velocity reports carry east-west and north-south components,
    # which are decoded to a ground speed and a track
    velocity = hex_to_bits("8D485020994409940838175B284F")
    vectors.append(TestVector([], [velocity], [frame_vector([
        {"df": 17, "icao": 0x485020, "hex": lua_string("8d485020994409940838175b284f"),
         "type_code": 19, "subtype": 1,
         "groundspeed": math.sqrt(8 * 8 + 159 * 159),
         "track": math.degrees(math.atan2(-8, -159)) % 360,
         "vertical_rate": -832, "vertical_rate_source": lua_string("gnss"),
         "gnss_baro_diff": 550}])],
        "Airborne velocity"))

    # An airborne position report whose altitude uses the legacy Q=0 (Gillham)
    # encoding, rather than the modern 25 foot encoding
    gillham = hex_to_bits("8D484175583232D690C8AC4CE22A")
    vectors.append(TestVector([], [gillham], [frame_vector([
        {"df": 17, "icao": 0x484175, "hex": lua_string("8d484175583232d690c8ac4ce22a"),
         "type_code": 11, "altitude": 50000, "surveillance_status": 0,
         "cpr_format": 0, "cpr_lat": 93000, "cpr_lon": 51372}])],
        "Airborne position, Q=0 (Gillham) altitude"))

    # A DF11 all-call reply has no extended squitter payload
    all_call_reply = hex_to_bits("5DABCDEF8A6AB3")
    vectors.append(TestVector([], [all_call_reply], [frame_vector([
        {"df": 11, "icao": 0xABCDEF, "hex": lua_string("5dabcdef8a6ab3")}])],
        "All-call reply"))

    # A frame with a bit error fails its syndrome check and is dropped
    corrupted = numpy.copy(identification)
    corrupted[20] = not corrupted[20]
    vectors.append(TestVector([], [corrupted], [frame_vector([])], "Frame with a bit error"))

    return BlockSpec("ADSBFramerBlock", vectors, 1e-6)
