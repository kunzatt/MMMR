package com.ssafy.mmmr.account.dto;

import io.swagger.v3.oas.annotations.media.Schema;
import jakarta.validation.constraints.Email;
import jakarta.validation.constraints.NotBlank;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
@Schema(description = "로그인 요청")
public class LogInRequestDto {

	@NotBlank(message = "이메일은 필수 입력값입니다.")
	@Email(message = "이메일 형식이 올바르지 않습니다.")
	@Schema(description = "로그인 이메일", example = "user@example.com", required = true)
	private String email;

	@NotBlank(message = "비밀번호는 필수 입력값입니다.")
	@Schema(description = "비밀번호", example = "password123!", required = true)
	private String password;
}