package com.ssafy.mmmr.global.error.exception;

import com.ssafy.mmmr.global.error.code.ErrorCode;

import lombok.Getter;

@Getter
public class UserException extends BusinessException {
	public UserException(ErrorCode errorCode) {
		super(errorCode);
	}
}

