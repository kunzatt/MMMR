package com.ssafy.mmmr.account.controller;

import java.util.HashMap;
import java.util.Map;

import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;

import com.ssafy.mmmr.account.dto.SignUpReqeustDto;
import com.ssafy.mmmr.account.service.AccountService;
import com.ssafy.mmmr.global.error.code.ErrorCode;
import com.ssafy.mmmr.global.error.dto.ErrorResponse;
import com.ssafy.mmmr.global.error.exception.AccountException;
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
@RequestMapping("/api/accounts")
@RequiredArgsConstructor
@Slf4j
@Tag(name = "계정 관리", description = "회원가입, 비밀번호 변경 관련 API")
public class AccountController {

	private final AccountService accountService;

	@PostMapping("/")
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

	@GetMapping("/email-exists")
	@Operation(summary = "이메일 중복 확인", description = "이메일 주소의 중복 여부를 확인합니다.")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "200",
			description = "이메일 중복 확인 결과",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
				{
					"message": "이메일 중복 확인 성공",
					"data": {
						"exists": false
					}
				}
				""")
			)
		)
	})
	public ResponseEntity<ApiResponse> checkEmailExists(
		@Parameter(description = "중복 확인할 이메일", required = true)
		@RequestParam String email
	) {
		try {
			accountService.checkEmailExists(email);
			Map<String, Boolean> result = new HashMap<>();
			result.put("exists", false);
			return ResponseEntity.ok(new ApiResponse("사용 가능한 이메일입니다", result));
		} catch (AccountException e) {
			if (e.getErrorCode() == ErrorCode.EMAIL_EXIST) {
				Map<String, Boolean> result = new HashMap<>();
				result.put("exists", true);
				return ResponseEntity.ok(new ApiResponse("이미 사용 중인 이메일입니다", result));
			}
			throw e;
		}
	}

}