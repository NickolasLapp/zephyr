#ifndef LLL_MFI_AUDIO_INTERNAL_H
#define LLL_MFI_AUDIO_INTERNAL_H

// *** Whisper added for MFI.  Register callbacks for triggering the audio
// sync signal to the ezairo
typedef void (*mfi_audio_sync_assert_cb_fn)(void);
typedef void (*mfi_audio_sync_deassert_cb_fn)(void);
void lll_mfi_audio_sync_register_cb(mfi_audio_sync_assert_cb_fn mfi_audio_sync_assert_cb,
                                    mfi_audio_sync_deassert_cb_fn mfi_audio_sync_deassert_cb);

#endif // LLL_MFI_AUDIO_INTERNAL_H
