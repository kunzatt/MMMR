package com.ssafy.mmmr.global.error.exception;

import com.ssafy.mmmr.global.error.code.ErrorCode;

import lombok.Getter;

@Getter
public class JwtException extends BusinessException {
	public JwtException(ErrorCode errorCode) {
		super(errorCode);
	}
}
