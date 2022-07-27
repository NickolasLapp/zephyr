#ifndef ULL_MFI_AUDIO_INTERNAL_H
#define ULL_MFI_AUDIO_INTERNAL_H

typedef enum {
  MFI_AUDIO_SLOT_TYPE_PRIMARY,
  MFI_AUDIO_SLOT_TYPE_RETRANSMIT
} mfi_audio_slot_type_t;

// *** Whisper added for MFI.  Register a callback for when audio packets are
// received so the BLE stack knows where to pass them
typedef void (*mfi_audio_recv_cb_fn)(const uint8_t *data, uint8_t data_len,
                                     mfi_audio_slot_type_t slot_type);
void ull_mfi_audio_register_cb(mfi_audio_recv_cb_fn mfi_audio_cb);

#endif // ULL_MFI_AUDIO_INTERNAL_H
