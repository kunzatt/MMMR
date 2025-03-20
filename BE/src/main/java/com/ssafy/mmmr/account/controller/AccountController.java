package com.ssafy.mmmr.account.controller;

import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import com.ssafy.mmmr.account.dto.AccessTokenResponseDto;
import com.ssafy.mmmr.account.dto.LogInRequestDto;
import com.ssafy.mmmr.account.dto.LogInResponseDto;
import com.ssafy.mmmr.account.dto.SignUpReqeustDto;
import com.ssafy.mmmr.account.dto.TokenRequestDto;
import com.ssafy.mmmr.account.service.AccountService;
import com.ssafy.mmmr.global.error.dto.ErrorResponse;
import com.ssafy.mmmr.global.response.ApiResponse;

import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.Parameter;
import io.swagger.v3.oas.annotations.media.Content;
import io.swagger.v3.oas.annotations.media.ExampleObject;
import io.swagger.v3.oas.annotations.media.Schema;
import io.swagger.v3.oas.annotations.responses.ApiResponses;
import io.swagger.v3.oas.annotations.tags.Tag;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;

@RestController
@RequestMapping("/api/account")
@RequiredArgsConstructor
@Slf4j
@Tag(name = "계정 관리", description = "회원가입, 로그인, 토큰 관리 API")
public class AccountController {

	private final AccountService accountService;

	@PostMapping("/signup")
	@Operation(summary = "회원가입", description = "신규 사용자를 등록합니다.")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "201",
			description = "회원가입 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
					{
						"message": "회원가입 성공"
					}
					""")
			)
		),
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "400",
			description = "이미 존재하는 이메일",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = @ExampleObject(value = """
					{
						"timestamp": "2024-03-20T10:00:00",
						"status": 400,
						"message": "이미 존재하는 이메일입니다",
						"errors": []
					}
					""")
			)
		)
	})
	public ResponseEntity<ApiResponse> signUp(
		@Parameter(description = "회원가입 정보", required = true)
		@RequestBody SignUpReqeustDto signUpReqeustDto
	) {
		accountService.signUp(signUpReqeustDto);
		return ResponseEntity.status(HttpStatus.CREATED)
			.body(new ApiResponse("회원가입 성공", null));
	}

	@PostMapping("/login")
	@Operation(summary = "로그인", description = "이메일과 비밀번호로 로그인하여 액세스 토큰과 리프레시 토큰을 발급받습니다.")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "200",
			description = "로그인 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
					{
						"message": "로그인 성공",
						"data": {
							"accessToken": "eyJhbGciOiJIUzUxMiJ9...",
							"refreshToken": "eyJhbGciOiJIUzUxMiJ9..."
						}
					}
					""")
			)
		),
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "404",
			description = "계정을 찾을 수 없음",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = @ExampleObject(value = """
					{
						"timestamp": "2024-03-20T10:00:00",
						"status": 404,
						"message": "계정을 찾을 수 없습니다",
						"errors": []
					}
					""")
			)
		),
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "400",
			description = "비밀번호가 일치하지 않음",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = @ExampleObject(value = """
					{
						"timestamp": "2024-03-20T10:00:00",
						"status": 400,
						"message": "비밀번호가 일치하지 않습니다",
						"errors": []
					}
					""")
			)
		)
	})
	public ResponseEntity<ApiResponse> login(
		@Parameter(description = "로그인 정보", required = true)
		@RequestBody LogInRequestDto logInRequestDto
	) {
		String[] tokens = accountService.login(logInRequestDto);
		LogInResponseDto loginResponseDto = LogInResponseDto.builder()
			.accessToken(tokens[0])
			.refreshToken(tokens[1])
			.build();

		return ResponseEntity.ok(new ApiResponse("로그인 성공", loginResponseDto));
	}

	@PostMapping("/refresh")
	@Operation(summary = "액세스 토큰 갱신", description = "리프레시 토큰을 사용해 새 액세스 토큰을 발급받습니다.")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "200",
			description = "토큰 리프레시 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
					{
						"message": "토큰 리프레시 성공",
						"data": {
							"accessToken": "eyJhbGciOiJIUzUxMiJ9..."
						}
					}
					""")
			)
		),
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "401",
			description = "유효하지 않은 토큰 또는 만료된 토큰",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = {
					@ExampleObject(
						name = "유효하지 않은 토큰",
						value = """
							{
								"timestamp": "2024-03-20T10:00:00",
								"status": 401,
								"message": "유효하지 않은 토큰입니다",
								"errors": []
							}
							"""
					),
					@ExampleObject(
						name = "만료된 토큰",
						value = """
							{
								"timestamp": "2024-03-20T10:00:00",
								"status": 401,
								"message": "만료된 토큰입니다",
								"errors": []
							}
							"""
					)
				}
			)
		)
	})
	public ResponseEntity<ApiResponse> refreshToken(
		@Parameter(description = "리프레시 토큰 정보", required = true)
		@RequestBody TokenRequestDto tokenDto
	) {
		String newAccessToken = accountService.refreshToken(tokenDto, null);
		return ResponseEntity.ok(new ApiResponse("토큰 리프레시 성공",
			new AccessTokenResponseDto(newAccessToken)));
	}

	@PostMapping("/logout")
	@Operation(summary = "로그아웃", description = "액세스 토큰을 무효화하고 리프레시 토큰을 삭제합니다.")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "200",
			description = "로그아웃 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
					{
						"message": "로그아웃 성공"
					}
					""")
			)
		),
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "401",
			description = "유효하지 않은 토큰",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = @ExampleObject(value = """
					{
						"timestamp": "2024-03-20T10:00:00",
						"status": 401,
						"message": "유효하지 않은 토큰입니다",
						"errors": []
					}
					""")
			)
		)
	})
	public ResponseEntity<ApiResponse> logout(
		@Parameter(description = "액세스 토큰 정보", required = true)
		@RequestBody TokenRequestDto tokenDto
	) {
		accountService.logoutWithToken(tokenDto);
		return ResponseEntity.ok(new ApiResponse("로그아웃 성공", null));
	}
}