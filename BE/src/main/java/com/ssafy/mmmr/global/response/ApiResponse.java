package com.ssafy.mmmr.global.response;

import lombok.AllArgsConstructor;
import lombok.Getter;

@Getter
@AllArgsConstructor
public class ApiResponse {
	private String message;
	private Object data;
}