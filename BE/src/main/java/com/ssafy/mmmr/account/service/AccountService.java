package com.ssafy.mmmr.account.service;

import org.springframework.security.crypto.password.PasswordEncoder;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import org.springframework.util.StringUtils;

import com.ssafy.mmmr.account.dto.LogInRequestDto;
import com.ssafy.mmmr.account.dto.SignUpReqeustDto;
import com.ssafy.mmmr.account.dto.TokenRequestDto;
import com.ssafy.mmmr.account.entity.AccountEntity;
import com.ssafy.mmmr.account.repository.AccountRepository;
import com.ssafy.mmmr.global.error.code.ErrorCode;
import com.ssafy.mmmr.global.error.exception.AccountException;
import com.ssafy.mmmr.global.error.exception.JwtException;
import com.ssafy.mmmr.jwt.provider.JwtTokenProvider;
import com.ssafy.mmmr.jwt.service.JwtRedisService;

import lombok.RequiredArgsConstructor;

@Service
@RequiredArgsConstructor
public class AccountService {

	private final AccountRepository accountRepository;
	private final PasswordEncoder passwordEncoder;
	private final JwtTokenProvider jwtTokenProvider;
	private final JwtRedisService jwtRedisService;

	@Transactional
	public void signUp(SignUpReqeustDto signUpReqeustDto) {
		accountRepository.findByEmail(signUpReqeustDto.getEmail())
			.ifPresent(accountEntity -> {
				throw new AccountException(ErrorCode.EMAIL_EXIST);
			});

		AccountEntity account = AccountEntity.builder()
			.email(signUpReqeustDto.getEmail())
			.password(passwordEncoder.encode(signUpReqeustDto.getPassword()))
			.address(signUpReqeustDto.getAddress())
			.build();

		accountRepository.save(account);
	}

	@Transactional
	public void checkEmailExists(String email) {
		if (accountRepository.existsByEmail(email)) {
			throw new AccountException(ErrorCode.EMAIL_EXIST);
		}
	}


}