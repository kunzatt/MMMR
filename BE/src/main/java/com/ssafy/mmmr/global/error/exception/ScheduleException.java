package com.ssafy.mmmr.global.error.exception;

import com.ssafy.mmmr.global.error.code.ErrorCode;

public class ScheduleException extends BusinessException {
	public ScheduleException(ErrorCode errorCode) {
		super(errorCode);
	}
}