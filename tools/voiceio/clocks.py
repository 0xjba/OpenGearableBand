"""Device audio clock constants for the gestureband wearable (nRF52840).

The speaker (I2S DAC) and the mic (PDM) are BOTH clocked from the same on-chip HFCLK
oscillator (via PCLK32M), so they are SYNCHRONOUS: their sample-rate ratio is fixed by
the divider configs and is invariant to temperature and unit-to-unit crystal variation
(temperature shifts HFCLK, which scales both clocks equally -> the ratio is unchanged).
This is the "same-SoC shared master clock" case, where production AEC uses a FIXED ratio
rather than continuous drift estimation -- the latter is only needed for ASYNCHRONOUS
multi-device setups (e.g. a Bluetooth speaker vs a phone) that have SEPARATE crystals.

  I2S speaker LRCK = 32 MHz / 21 / 96 = 15873.0 Hz   (nRF I2S MCKFREQ_32MDIV21 / RATIO_96X;
                                                      it can't hit 16000 exactly -- integer divider)
  PDM mic rate    = 16000 Hz                          (decimated PDM clock; the ts32 full-span
                                                      measured 15999.9 Hz, confirming it)

AEC_DRIFT_RATIO is what voiceio.resample uses to put the far-end reference on the mic
(PDM) clock so the echo aligns: the echo is PLAYED at the I2S rate and CAPTURED at the PDM
rate, so the reference must be scaled by I2S/PDM. Verified on hardware (drift1): the fixed
ratio holds the echo removed start-to-end, and an independent blind sweep found the best
alignment at -0.80%, matching 15873/16000 = -0.794%.  [STRUCTURAL: re-derive if the I2S or
PDM clock config changes in firmware.]
"""
I2S_HZ = 15873.0                       # speaker DAC LRCK (32e6 / 21 / 96)
PDM_HZ = 16000.0                       # mic PCM rate
AEC_DRIFT_RATIO = I2S_HZ / PDM_HZ      # 0.99206 (-0.794%): scale far-end ref onto the mic clock
