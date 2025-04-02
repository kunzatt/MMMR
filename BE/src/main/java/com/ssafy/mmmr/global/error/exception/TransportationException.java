package com.ssafy.mmmr.global.error.exception;

import com.ssafy.mmmr.global.error.code.ErrorCode;

public class TransportationException extends BusinessException {
	public TransportationException(ErrorCode errorCode) {
		super(errorCode);
	}
}
