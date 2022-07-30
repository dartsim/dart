# Copyright (c) 2011-2022, The DART development contributors

import pytest
import dartpy8 as dart


def test_from_string_valid_uri_returns_true():
    uri = dart.common.Uri()
    assert uri.from_string("ftp://ftp.is.co.za/rfc/rfc1808.txt") is True
    assert uri.from_string("http://www.ietf.org/rfc/rfc2396.txt") is True
    assert uri.from_string("ldap://[2001:db8::7]/c=GB?objectClass?one") is True
    assert uri.from_string("mailto:John.Doe@example.com") is True
    assert uri.from_string("news:comp.infosystems.www.servers.unix") is True
    assert uri.from_string("tel:+1-816-555-1212") is True
    assert uri.from_string("telnet://192.0.2.16:80/") is True
    assert uri.from_string(
        "urn:oasis:names:specification:docbook:dtd:xml:4.1.2") is True


if __name__ == "__main__":
    pytest.main()
