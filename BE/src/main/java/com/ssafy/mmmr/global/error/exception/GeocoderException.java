package com.ssafy.mmmr.global.error.exception;

import com.ssafy.mmmr.global.error.code.ErrorCode;

public class GeocoderException extends BusinessException {
	public GeocoderException(ErrorCode errorCode) {
		super(errorCode);
	}
}
