package com.ssafy.mmmr.global.error.exception;

import com.ssafy.mmmr.global.error.code.ErrorCode;

public class HomeDeviceException extends BusinessException {
	public HomeDeviceException(ErrorCode errorCode) {
		super(errorCode);
	}
}
