package com.ssafy.mmmr.global.error.exception;

import com.ssafy.mmmr.global.error.code.ErrorCode;

public class VideoException extends BusinessException {
    public VideoException(ErrorCode errorCode) {
        super(errorCode);
    }
}
