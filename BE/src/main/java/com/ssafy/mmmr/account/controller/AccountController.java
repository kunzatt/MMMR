package com.ssafy.mmmr.account.controller;

import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.util.StringUtils;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestHeader;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import com.ssafy.mmmr.account.dto.LogInRequestDto;
import com.ssafy.mmmr.account.dto.LogInResponseDto;
import com.ssafy.mmmr.account.dto.SignUpReqeustDto;
import com.ssafy.mmmr.account.service.AccountService;
import com.ssafy.mmmr.global.response.ApiResponse;

import lombok.RequiredArgsConstructor;

@RestController
@RequestMapping("/api/account")
@RequiredArgsConstructor
public class AccountController {

	private final AccountService accountService;

	@PostMapping("/signup")
	public ResponseEntity<ApiResponse> signUp(@RequestBody SignUpReqeustDto signUpReqeustDto) {
		accountService.signUp(signUpReqeustDto);
		return ResponseEntity.status(HttpStatus.CREATED)
			.body(new ApiResponse("회원가입 성공", null));
	}

	@PostMapping("/login")
	public ResponseEntity<ApiResponse> login(@RequestBody LogInRequestDto logInRequestDto) {
		String[] tokens = accountService.login(logInRequestDto);
		LogInResponseDto loginResponseDto = LogInResponseDto.builder()
			.accessToken(tokens[0])
			.refreshToken(tokens[1])
			.build();

		return ResponseEntity.ok(new ApiResponse("로그인 성공", loginResponseDto));
	}

	@PostMapping("/refresh")
	public ResponseEntity<ApiResponse> refreshToken(@RequestHeader("Authorization") String refreshToken) {
		if (StringUtils.hasText(refreshToken) && refreshToken.startsWith("Bearer ")) {
			refreshToken = refreshToken.substring(7);
		}

		String newAccessToken = accountService.refresh(refreshToken);

		return ResponseEntity.ok(new ApiResponse("토큰 리프레시 성공",
			new LogInResponseDto(newAccessToken, null)));
	}

	@PostMapping("/logout")
	public ResponseEntity<ApiResponse> logout(@RequestHeader(value = "Authorization", required = false) String authHeader) {
		accountService.logout(authHeader);
		return ResponseEntity.ok(new ApiResponse("로그아웃 성공", null));
	}
}