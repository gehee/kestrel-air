# Spectator late joining

`allow_spectators` defaults to 1 in the air settings. The diagnostic control protocol accepts
command `0x45` with exactly one boolean byte; the air saves it and includes
`spectators=0/1` in its normal info report. Stock/older air software should not
receive this command. The native pilot UI only shows sharing details; it has no spectator toggle.
No new handshake with the spectator is required.

When enabled and streaming, the shared video loop requests a recovery picture
every two seconds, at a picture boundary. It does not change the configured GOP
or intra-refresh policy. The AR9311 backend requests an IDR from every strip
encoder. The HiSilicon backend uses its existing vendor IDR request API; that
backend still needs hardware validation. No catch-up burst occurs after a stall.
Disabling, disconnecting or stopping transmission resets the recovery timer.

The final slice also carries an HEVC suffix SEI at most four times per second:
user_data_unregistered, UUID ASCII `kestrel-spec-v1` padded to 16 bytes with
zeros; version u8=1; flags u8=1 (opt-in); radio MAC[4]; width, height, fps,
recovery interval in milliseconds, all unsigned LE16. Normal Annex-B emulation
prevention applies. Unknown versions/flags must be ignored. This is stream
metadata after RF acquisition, not a common-channel RF discovery beacon.

The setting is a discovery/recovery preference, not access control. Turning it
off cannot prevent another receiver from decoding an already-acquired stream.
Recovery frames can increase instantaneous bitrate; measure pilot latency and
loss under constrained links before choosing a release interval.
