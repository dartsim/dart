import platform

import pytest
from dartpy.common import Uri


def test_from_string_valid_uri_returns_true():
    uri = Uri()
    assert uri.fromString("ftp://ftp.is.co.za/rfc/rfc1808.txt") is True
    assert uri.fromString("http://www.ietf.org/rfc/rfc2396.txt") is True
    assert uri.fromString("ldap://[2001:db8::7]/c=GB?objectClass?one") is True
    assert uri.fromString("mailto:John.Doe@example.com") is True
    assert uri.fromString("news:comp.infosystems.www.servers.unix") is True
    assert uri.fromString("tel:+1-816-555-1212") is True
    assert uri.fromString("telnet://192.0.2.16:80/") is True
    assert uri.fromString("urn:oasis:names:specification:docbook:dtd:xml:4.1.2") is True


if __name__ == "__main__":
    pytest.main()
