package com.ssafy.mmmr.account.service;

import org.springframework.security.crypto.password.PasswordEncoder;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import org.springframework.util.StringUtils;

import com.ssafy.mmmr.account.dto.LogInRequestDto;
import com.ssafy.mmmr.account.dto.SignUpReqeustDto;
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

	@Transactional(readOnly = true)
	public String[] login(LogInRequestDto logInRequestDto) {

		AccountEntity account = accountRepository.findByEmail(logInRequestDto.getEmail())
			.orElseThrow(() -> new AccountException(ErrorCode.ACCOUNT_NOT_FOUND));

		if (!passwordEncoder.matches(logInRequestDto.getPassword(), account.getPassword())) {
			throw new AccountException(ErrorCode.INVALID_PASSWORD);
		}

		String accessToken = jwtTokenProvider.generateAccessToken(account.getEmail());
		String refreshToken = jwtTokenProvider.generateRefreshToken(account.getEmail());

		jwtRedisService.saveRefreshToken(account.getEmail(), refreshToken,
			jwtTokenProvider.getRemainingTimeFromToken(refreshToken));

		return new String[] { accessToken, refreshToken };
	}

	@Transactional
	public String refresh(String refreshToken) {
		jwtTokenProvider.validateToken(refreshToken);

		String email = jwtTokenProvider.getEmailFromToken(refreshToken);

		String token = jwtRedisService.getRefreshToken(email);
		if (token == null || !token.equals(refreshToken)) {
			throw new JwtException(ErrorCode.INVALID_TOKEN);
		}

		return jwtTokenProvider.generateRefreshToken(email);
	}

	@Transactional
	public void logout(String authHeader) {
		// 헤더 검증
		if (!StringUtils.hasText(authHeader)) {
			throw new JwtException(ErrorCode.INVALID_TOKEN);
		}

		// Bearer 토큰 처리
		String accessToken = authHeader.startsWith("Bearer ")
			? authHeader.substring(7)
			: authHeader;

		// 토큰에서 이메일 추출
		String email = jwtTokenProvider.getEmailFromToken(accessToken);

		// 토큰의 남은 유효 시간 계산
		long remainingTime = jwtTokenProvider.getRemainingTimeFromToken(accessToken);

		// 블랙리스트에 토큰 추가
		jwtRedisService.addToBlacklist(accessToken, remainingTime);

		// 리프레시 토큰 삭제
		jwtRedisService.deleteRefreshToken(email);
	}
}
