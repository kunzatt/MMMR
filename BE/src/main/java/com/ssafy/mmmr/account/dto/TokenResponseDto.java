package com.ssafy.mmmr.account.dto;

import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
@AllArgsConstructor
@Schema(description = "토큰 응답")
public class TokenResponseDto {

	@Schema(description = "액세스 토큰", example = "eyJhbGciOiJIUzUxMiJ9...")
	private String accessToken;

	@Schema(description = "리프레시 토큰", example = "eyJhbGciOiJIUzUxMiJ9...")
	private String refreshToken;
}