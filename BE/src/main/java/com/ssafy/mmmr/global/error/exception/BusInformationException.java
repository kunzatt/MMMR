package com.ssafy.mmmr.global.error.exception;

import com.ssafy.mmmr.global.error.code.ErrorCode;

import lombok.Getter;

@Getter
public class BusInformationException extends BusinessException {
	public BusInformationException(ErrorCode errorCode) {
		super(errorCode);
	}
}
