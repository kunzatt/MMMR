package com.ssafy.mmmr.global.error.exception;

import com.ssafy.mmmr.global.error.code.ErrorCode;

public class WeatherException extends BusinessException {
	public WeatherException(ErrorCode errorCode) {
		super(errorCode);
	}
}
