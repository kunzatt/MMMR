package com.ssafy.mmmr.global.error.exception;

import com.ssafy.mmmr.global.error.code.ErrorCode;

public class ProfileException extends BusinessException {
	public ProfileException(ErrorCode errorCode) {
		super(errorCode);
	}
}
