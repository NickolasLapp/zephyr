#ifndef ULL_MFI_AUDIO_INTERNAL_H
#define ULL_MFI_AUDIO_INTERNAL_H

// *** Whisper added for MFI.  Register a callback for when audio packets are
// received so the BLE stack knows where to pass them
typedef void (*mfi_audio_recv_cb_fn)(const uint8_t *data, uint8_t data_len);
void ull_mfi_audio_register_cb(mfi_audio_recv_cb_fn mfi_audio_cb);

#endif // ULL_MFI_AUDIO_INTERNAL_H
