package com.ssafy.mmmr.global.error.exception;

import com.ssafy.mmmr.global.error.code.ErrorCode;

public class PasswordUpdateException extends BusinessException {
	public PasswordUpdateException(ErrorCode errorCode) {
		super(errorCode);
	}
}
