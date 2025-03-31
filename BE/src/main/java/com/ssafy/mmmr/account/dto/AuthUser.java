package com.ssafy.mmmr.account.dto;

import com.ssafy.mmmr.account.entity.AccountEntity;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class AuthUser {

	private Long id;
	private String email;
	private String address;

	public static AuthUser from(AccountEntity accountEntity) {
		return AuthUser.builder()
			.id(accountEntity.getId())
			.email(accountEntity.getEmail())
			.address(accountEntity.getAddress())
			.build();
	}

}
