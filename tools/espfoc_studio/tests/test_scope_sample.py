from espfoc_studio.link.scope_sample import (
    decode_scope_payload_to_floats,
    decode_scope_payload_to_floats_csv_first,
    pack_scope_i32_to_payload,
)


def test_bin_round_trip() -> None:
    pl = pack_scope_i32_to_payload(
        [65536, 2 * 65536, 0, -131072, 0, 0, 0, 0]
    )
    v = decode_scope_payload_to_floats(pl)
    assert v is not None
    assert len(v) == 8
    assert abs(v[0] - 1.0) < 1e-5
    assert abs(v[1] - 2.0) < 1e-5
    assert v[2] == 0.0
    assert abs(v[3] - -2.0) < 1e-5


def test_csv_only_when_env_enabled(monkeypatch) -> None:
    a = b"0.1,0.2,0.3\n"
    assert decode_scope_payload_to_floats_csv_first(a) == []
    monkeypatch.setenv("ESP_FOC_STUDIO_SCOPE_CSV", "1")
    w = decode_scope_payload_to_floats_csv_first(a)
    assert w == [0.1, 0.2, 0.3]
