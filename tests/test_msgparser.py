import msgparser as parser
from nose.tools import *

@raises(AssertionError)
def test_null_msg():
    parser.MsgParser.pack_data([])

def test_pack_multiple_messages():
    x = [1]
    y = range(255)
    z = range(23)

    lenx = len(x)
    leny = len(y)
    lenz = len(z)

    packed_x = parser.MsgParser.pack_data(x)
    packed_y = parser.MsgParser.pack_data(y)
    packed_z = parser.MsgParser.pack_data(z)

    assert(packed_x[0] == parser.MsgParser.START)
    assert(packed_x[-1] == parser.MsgParser.END)

    assert((packed_x[1] | packed_x[2]) == lenx)
    assert((packed_y[1] | packed_y[2]) == leny)
    assert((packed_z[1] | packed_z[2]) == lenz)

@raises(AssertionError)
def test_pack_msg_too_long():
    x = []
    for i in range(0xffff + 1):
        x.append(1) # Fill list with ones to avoid raising exception in bytearray.
    parser.MsgParser.pack_data(x)

def test_valid_message():
    data = range(255)
    packed_data = parser.MsgParser.pack_data(data)
    p = parser.MsgParser()
    for d in packed_data:
        p.parse(d)
    assert(p.has_message)
    parsed_data = p.get_message()
    assert(not p.has_message)
    assert(parsed_data == data)

@raises(RuntimeError)
def test_parse_invalid_message_lenght():
    data = range(10)
    packed_data = parser.MsgParser.pack_data(data)
    packed_data.insert(3, 0) # Insert data right after lengh bytes.
    p = parser.MsgParser()
    for d in packed_data:
        p.parse(d)
