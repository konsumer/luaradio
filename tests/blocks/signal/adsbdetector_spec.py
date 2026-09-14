import numpy
from generate import *


def hex_to_bits(hexstr):
    return numpy.array([int(c, 16) >> s & 1 for c in hexstr for s in (3, 2, 1, 0)], dtype=numpy.bool_)


def waveform(symbol_period, bits, start, pad=600):
    """Generate the envelope of a preamble and PPM frame whose first preamble
    sample sits at input sample `start`, sampled at n / symbol_period symbols.
    Trailing samples are padded, since the block only evaluates a candidate
    once a full length frame of samples has been buffered."""
    length = int(numpy.ceil(start + (8 + len(bits)) * symbol_period)) + pad
    x = numpy.zeros(length, dtype=numpy.float32)

    for n in range(x.size):
        t = (n - start) / symbol_period

        if t < 0:
            continue

        if t < 8.0:
            for pulse in (0.0, 1.0, 3.5, 4.5):
                if pulse <= t < pulse + 0.5:
                    x[n] = 1.0
        else:
            u = t - 8.0
            i = int(numpy.floor(u))
            if i < len(bits) and (bits[i] == (u - i < 0.5)):
                x[n] = 1.0

    return x


def generate():
    vectors = []

    # A DF17 extended squitter, a DF17 airborne position, and a DF11 all-call reply
    squitter = hex_to_bits("8D4840D6202CC371C32CE0576098")
    position = hex_to_bits("8D40621D58C382D690C8AC2863A7")
    all_call_reply = hex_to_bits("5DABCDEF8A6AB3")

    # symbol_rate 1.0 against the normalized test sample rate of 2.0 gives an
    # integer symbol period of 2 samples, so frames fall on the sample grid
    vectors.append(TestVector([1.0], [waveform(2.0, squitter, 12.0)], [squitter],
                              "2 samples per symbol, 112 bit frame"))

    # The downlink format of the frame selects its length
    vectors.append(TestVector([1.0], [waveform(2.0, all_call_reply, 12.0)], [all_call_reply],
                              "2 samples per symbol, 56 bit frame"))

    # Consecutive frames
    x = numpy.hstack((waveform(2.0, squitter, 12.0), waveform(2.0, all_call_reply, 12.0)))
    vectors.append(TestVector([1.0], [x], [numpy.hstack((squitter, all_call_reply))],
                              "2 samples per symbol, two frames"))

    # symbol_rate 0.5 gives a symbol period of 4 samples, and the frame starts
    # on the resampled grid
    vectors.append(TestVector([0.5], [waveform(4.0, squitter, 16.0)], [squitter],
                              "4 samples per symbol, 112 bit frame"))

    # The same frame starting half a sample off the resampled grid is recovered
    # by the phase correction retry
    vectors.append(TestVector([0.5], [waveform(4.0, squitter, 17.0)], [squitter],
                              "4 samples per symbol, off phase frame"))

    # A frame with a bit error in the payload is corrected from its syndrome
    corrupted = numpy.copy(position)
    corrupted[40] = not corrupted[40]
    vectors.append(TestVector([1.0], [waveform(2.0, corrupted, 12.0)], [position],
                              "2 samples per symbol, one bit error corrected"))

    # Two bit errors are corrected as well
    corrupted = numpy.copy(position)
    corrupted[40] = not corrupted[40]
    corrupted[70] = not corrupted[70]
    vectors.append(TestVector([1.0], [waveform(2.0, corrupted, 12.0)], [position],
                              "2 samples per symbol, two bit errors corrected"))

    return BlockSpec("ADSBDetectorBlock", vectors, 1e-6)
