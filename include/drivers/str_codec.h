#pragma once

struct str_codec;
typedef struct str_codec * str_codec_handle;

int codec_init(str_codec_handle * pHandle);

int codec_test_case_1();
int codec_test_case_2();

enum CODEC_UPDATE_TYPE {
  CODEC_UPDATE_ALL,
  CODEC_UPDATE_TX1,
  CODEC_UPDATE_RX1,
  CODEC_UPDATE_TX2,
  CODEC_UPDATE_RX2,
  CODEC_UPDATE_TX1_RX1,
  CODEC_UPDATE_TX2_RX2,
};

int32_t Codec_update(enum CODEC_UPDATE_TYPE type, ...);
