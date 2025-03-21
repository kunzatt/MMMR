package com.ssafy.mmmr.account.dto;

import io.swagger.v3.oas.annotations.media.Schema;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
@Schema(description = "비밀번호 변경")
public class PasswordUpdateRequestDto {

	private String currentPassword;
	private String newPassword;
	private String newPasswordConfirm;
}
