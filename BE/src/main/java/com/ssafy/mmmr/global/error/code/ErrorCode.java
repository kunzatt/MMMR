package com.ssafy.mmmr.global.error.code;

import lombok.AllArgsConstructor;
import lombok.Getter;

@Getter
@AllArgsConstructor
public enum ErrorCode {
	// Common
	INVALID_INPUT_VALUE(400, "잘못된 입력값입니다"),
	INTERNAL_SERVER_ERROR(500, "서버 오류가 발생했습니다"),
	UNAUTHORIZED(401, "인증되지 않은 접근입니다"),
	FORBIDDEN(403, "권한이 없습니다"),
	INVALID_ENUM_VALUE(400, "잘못된 상태값입니다"),

	// Auth & User
	INVALID_TOKEN(401, "유효하지 않은 토큰입니다"),
	EXPIRED_TOKEN(401, "만료된 토큰입니다"),
	REFRESH_TOKEN_NOT_FOUND(401, "리프레시 토큰을 찾을 수 없습니다."),
	EMAIL_EXIST(400, "이미 존재하는 이메일입니다"),
	INVALID_PASSWORD(400, "잘못된 비밀번호입니다"),
	LOGIN_BAD_CREDENTIALS(401, "이메일 또는 비밀번호가 일치하지 않습니다."),
	LOGIN_FAILED(401, "로그인에 실패했습니다. 이메일과 비밀번호를 확인해주세요."),
	PASSWORD_MISMATCH(400, "새 비밀번호가 일치하지 않습니다"),
	USERNAME_EMAIL_MISMATCH(400, "사용자 이름과 이메일이 일치하지 않습니다"),

	// BusInformation
	INVALID_EXTENSION_VALUE(400, "올바른 확장자 명이 아닙니다"),
	FILE_UPLOAD_ERROR(500, "파일 업로드 중 오류가 발생했습니다"),
	BATCH_PROCESSING_ERROR(500, "배치 작업 실행 중 오류가 발생했습니다"),

	// Excel File
	EMPTY_FILE(400, "파일이 비어있습니다"),
	EXCEL_PROCESSING_ERROR(500, "엑셀 파일 처리 중 오류가 발생했습니다");

	private final int status;
	private final String message;

}
