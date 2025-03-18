package com.ssafy.mmmr.global.error.exception;

import com.ssafy.mmmr.global.error.code.ErrorCode;

public class NewsException extends BusinessException {
    public NewsException(ErrorCode errorCode) { super(errorCode); }
}
