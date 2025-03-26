package com.ssafy.mmmr.account.dto;

import jakarta.validation.constraints.NotBlank;
import jakarta.validation.constraints.Pattern;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
public class VerifyCodeRequestDto {

	@NotBlank(message = "인증코드는 필수입니다.")
	@Pattern(regexp = "^[0-9]{6}$", message = "인증코드는 6자리 숫자여야 합니다.")
	private String code;
}
