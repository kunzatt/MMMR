package com.ssafy.mmmr.global.error.exception;

import com.ssafy.mmmr.global.error.code.ErrorCode;

public class TodoException extends BusinessException {
	public TodoException(ErrorCode errorCode) {
		super(errorCode);
	}
}
