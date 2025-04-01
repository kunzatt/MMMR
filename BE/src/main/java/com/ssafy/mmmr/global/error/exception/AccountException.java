package com.ssafy.mmmr.global.error.exception;

import com.ssafy.mmmr.global.error.code.ErrorCode;

import lombok.Getter;

@Getter
public class AccountException extends BusinessException {
	public AccountException(ErrorCode errorCode) {
		super(errorCode);
	}
}

