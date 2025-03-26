package com.ssafy.mmmr.global.error.exception;

import com.ssafy.mmmr.global.error.code.ErrorCode;

public class EmailException extends BusinessException {
  public EmailException(ErrorCode errorCode) {
    super(errorCode);
  }
}
