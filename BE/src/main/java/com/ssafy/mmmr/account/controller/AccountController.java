package com.ssafy.mmmr.account.controller;

import java.util.Map;

import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.DeleteMapping;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.PutMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;

import com.ssafy.mmmr.account.dto.AuthUser;
import com.ssafy.mmmr.account.dto.DeleteAccountRequestDto;
import com.ssafy.mmmr.account.dto.PasswordUpdateRequestDto;
import com.ssafy.mmmr.account.dto.SignUpReqeustDto;
import com.ssafy.mmmr.account.service.AccountService;
import com.ssafy.mmmr.global.annotation.CurrentUser;
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
@RequestMapping("/api/accounts")
@RequiredArgsConstructor
@Tag(name = "Account 관리", description = "회원가입, 비밀번호 변경 관련 API")
public class AccountController {

	private final AccountService accountService;

	@PostMapping
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
		Map<String, Boolean> result = accountService.checkEmailExists(email);

		String message = result.get("exists") ?
			"이미 사용 중인 이메일입니다" :
			"사용 가능한 이메일입니다";

		return ResponseEntity.ok(new ApiResponse(message, result));
	}

	@PutMapping("/password")
	@Operation(summary = "비밀번호 변경", description = "사용자의 비밀번호를 변경합니다.")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "200",
			description = "비밀번호 변경 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "message": "비밀번호 변경 성공"
                    }
                    """)
			)
		),
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "400",
			description = "비밀번호 변경 실패",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "timestamp": "2024-03-20T10:00:00",
                        "status": 400,
                        "message": "현재 비밀번호가 일치하지 않습니다",
                        "errors": []
                    }
                    """)
			)
		),
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "401",
			description = "인증 정보 없음",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = @ExampleObject(value = """
                    {
                        "timestamp": "2024-03-20T10:00:00",
                        "status": 401,
                        "message": "인증 정보가 없습니다",
                        "errors": []
                    }
                    """)
			)
		)
	})
	public ResponseEntity<ApiResponse> updatePassword(
		@Parameter(hidden = true) @CurrentUser AuthUser authUser,
		@io.swagger.v3.oas.annotations.parameters.RequestBody(
			description = "비밀번호 변경 정보",
			required = true,
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = PasswordUpdateRequestDto.class),
				examples = @ExampleObject(
					value = """
                    {
                        "currentPassword": "현재비밀번호123!",
                        "newPassword": "새비밀번호123!",
                        "newPasswordConfirm": "새비밀번호123!"
                    }
                    """
				)
			)
		)
		@Valid @RequestBody PasswordUpdateRequestDto request
	) {

		if (authUser == null) {
			return ResponseEntity.status(HttpStatus.UNAUTHORIZED)
				.body(new ApiResponse("인증 정보가 없습니다", null));
		}

		if (authUser.getId() == null) {
			return ResponseEntity.status(HttpStatus.BAD_REQUEST)
				.body(new ApiResponse("사용자 정보가 올바르지 않습니다", null));
		}

		accountService.updatePassword(
			authUser.getId(),
			request.getCurrentPassword(),
			request.getNewPassword(),
			request.getNewPasswordConfirm()
		);

		return ResponseEntity.ok(new ApiResponse("비밀번호 변경 성공", null));
	}

	@DeleteMapping
	@Operation(summary = "회원 탈퇴", description = "비밀번호 확인 후 회원을 탈퇴합니다")
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "200",
			description = "회원 탈퇴 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
                {
                    "message": "성공적으로 회원 탈퇴하였습니다.."
                }
                """)
			)
		),
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "400",
			description = "회원 탈퇴 실패",
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
		),
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "401",
			description = "인증 정보 없음",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = @ExampleObject(value = """
                {
                    "timestamp": "2024-03-20T10:00:00",
                    "status": 401,
                    "message": "인증 정보가 없습니다",
                    "errors": []
                }
                """)
			)
		)
	})
	public ResponseEntity<ApiResponse> deleteAccount(
		@Parameter(hidden = true) @CurrentUser AuthUser authUser,
		@io.swagger.v3.oas.annotations.parameters.RequestBody(
			description = "회원 탈퇴 확인을 위한 비밀번호",
			required = true,
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = DeleteAccountRequestDto.class),
				examples = @ExampleObject(
					value = """
                {
                    "password": "현재비밀번호123!"
                }
                """
				)
			)
		)
		@Valid @RequestBody DeleteAccountRequestDto request
	) {
		accountService.deleteAccount(authUser, request.getPassword());
		return ResponseEntity.ok(new ApiResponse("회원 탈퇴를 성공하였습니다.", null));
	}
}