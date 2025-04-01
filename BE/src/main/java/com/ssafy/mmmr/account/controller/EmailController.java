package com.ssafy.mmmr.account.controller;

import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import com.ssafy.mmmr.account.dto.PasswordResetEmailRequestDto;
import com.ssafy.mmmr.account.dto.SendEmailRequestDto;
import com.ssafy.mmmr.account.dto.VerifyCodeRequestDto;
import com.ssafy.mmmr.account.service.EmailService;
import com.ssafy.mmmr.global.error.dto.ErrorResponse;
import com.ssafy.mmmr.global.response.ApiResponse;

import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.Parameter;
import io.swagger.v3.oas.annotations.media.Content;
import io.swagger.v3.oas.annotations.media.ExampleObject;
import io.swagger.v3.oas.annotations.media.Schema;
import io.swagger.v3.oas.annotations.responses.ApiResponses;
import io.swagger.v3.oas.annotations.tags.Tag;
import jakarta.validation.Valid;
import lombok.RequiredArgsConstructor;

@RestController
@RequestMapping("/api/mail")
@RequiredArgsConstructor
@Tag(name = "이메일 관리", description = "이메일 인증 및 비밀번호 찾기 관련 API")
public class EmailController {

	private final EmailService emailService;

	@PostMapping("/codes")
	@Operation(summary = "인증 코드 전송", description = "이메일 인증을 위한 코드를 전송합니다.")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "200",
			description = "이메일 전송 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
					{
						"message": "이메일을 전송 했습니다."
					}
					""")
			)
		),
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "400",
			description = "이메일 전송 실패",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = @ExampleObject(value = """
					{
						"timestamp": "2024-03-20T10:00:00",
						"status": 400,
						"message": "유효하지 않은 이메일 주소입니다",
						"errors": []
					}
					""")
			)
		)
	})
	public ResponseEntity<ApiResponse> sendEmailCode(
		@Parameter(description = "인증 코드를 전송할 이메일 정보", required = true)
		@RequestBody @Valid SendEmailRequestDto sendEmailRequestDto
	) {
		emailService.sendEmailCode(sendEmailRequestDto.getEmail());
		return ResponseEntity.ok(new ApiResponse("이메일을 전송 했습니다.", null));
	}

	@PostMapping("/verification/{email}")
	@Operation(summary = "인증 코드 확인", description = "사용자가 입력한 인증 코드의 유효성을 검증합니다.")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "200",
			description = "인증 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
					{
						"message": "인증 완료 됐습니다."
					}
					""")
			)
		),
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "400",
			description = "인증 실패",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = @ExampleObject(value = """
					{
						"timestamp": "2024-03-20T10:00:00",
						"status": 400,
						"message": "인증 코드가 일치하지 않습니다",
						"errors": []
					}
					""")
			)
		)
	})
	public ResponseEntity<ApiResponse> verifyCode(
		@Parameter(description = "인증할 이메일 주소", required = true)
		@PathVariable String email,
		@Parameter(description = "사용자가 입력한 인증 코드", required = true)
		@RequestBody @Valid VerifyCodeRequestDto verifyCodeRequestDto
	) {
		emailService.verifyCode(email, verifyCodeRequestDto.getCode());
		return ResponseEntity.ok(new ApiResponse("인증 완료 됐습니다.", null));
	}

	@PostMapping("/password")
	@Operation(summary = "임시 비밀번호 발송", description = "사용자에게 임시 비밀번호를 이메일로 전송합니다.")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "200",
			description = "임시 비밀번호 발송 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
					{
						"message": "임시 비밀번호가 이메일로 전송되었습니다."
					}
					""")
			)
		),
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "404",
			description = "사용자를 찾을 수 없음",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = @ExampleObject(value = """
					{
						"timestamp": "2024-03-20T10:00:00",
						"status": 404,
						"message": "해당 이메일로 등록된 사용자를 찾을 수 없습니다",
						"errors": []
					}
					""")
			)
		)
	})
	public ResponseEntity<ApiResponse> sendTemporaryPassword(
		@Parameter(description = "임시 비밀번호를 받을 이메일", required = true)
		@RequestBody @Valid PasswordResetEmailRequestDto passwordResetEmailRequestDto
	){
		emailService.sendTemporaryPassword(passwordResetEmailRequestDto.getEmail());
		return ResponseEntity.ok(new ApiResponse("임시 비밀번호가 이메일로 전송되었습니다.", null));
	}

}