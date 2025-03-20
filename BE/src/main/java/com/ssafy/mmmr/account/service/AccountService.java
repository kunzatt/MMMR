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

	private String processTokenFromDto(TokenRequestDto tokenRequestDto) {
		if (tokenRequestDto == null || tokenRequestDto.getToken() == null || tokenRequestDto.getToken().trim().isEmpty()) {
			throw new JwtException(ErrorCode.INVALID_TOKEN);
		}

		String token = tokenRequestDto.getToken();

		// Bearer 접두사 처리
		if (token.startsWith("Bearer ")) {
			token = token.substring(7);
		}

		return token;
	}

	@Transactional
	public String refreshToken(TokenRequestDto tokenDto, String oldAccessToken) {
		String refreshToken = processTokenFromDto(tokenDto);

		// 이전 액세스 토큰이 있는 경우 블랙리스트에 추가
		if (oldAccessToken != null && !oldAccessToken.isEmpty()) {
			// Bearer 접두사 제거
			if (oldAccessToken.startsWith("Bearer ")) {
				oldAccessToken = oldAccessToken.substring(7);
			}

			// 토큰이 유효한지 확인 (예외없이 확인)
			if (jwtTokenProvider.isTokenValid(oldAccessToken)) {
				// 토큰의 남은 유효 시간 계산
				long remainingTime = jwtTokenProvider.getRemainingTimeFromToken(oldAccessToken);

				// 블랙리스트에 토큰 추가
				jwtRedisService.addToBlacklist(oldAccessToken, remainingTime);
				System.out.println("이전 액세스 토큰을 블랙리스트에 추가했습니다.");
			} else {
				System.out.println("이전 액세스 토큰이 유효하지 않아 블랙리스트에 추가하지 않았습니다.");
			}
		}

		return refresh(refreshToken);
	}

	@Transactional
	public String refresh(String refreshToken) {
		// 토큰 유효성 검증
		if (!jwtTokenProvider.isTokenValid(refreshToken)) {
			throw new JwtException(ErrorCode.INVALID_TOKEN);
		}

		// 토큰에서 이메일 추출
		String email = jwtTokenProvider.getEmailFromToken(refreshToken);
		System.out.println("리프레시 토큰에서 추출한 이메일: " + email);

		// Redis에 저장된 리프레시 토큰 조회
		String storedToken = jwtRedisService.getRefreshToken(email);
		System.out.println("Redis에 저장된 리프레시 토큰: " + storedToken);

		// 토큰 비교
		if (storedToken == null) {
			System.out.println("저장된 리프레시 토큰이 없습니다");
			throw new JwtException(ErrorCode.INVALID_TOKEN);
		}

		if (!storedToken.equals(refreshToken)) {
			System.out.println("토큰이 일치하지 않습니다");
			throw new JwtException(ErrorCode.INVALID_TOKEN);
		}

		// 새 액세스 토큰 생성 및 반환
		return jwtTokenProvider.generateAccessToken(email);
	}

	@Transactional
	public void logoutWithToken(TokenRequestDto tokenRequestDto) {
		String accessToken = processTokenFromDto(tokenRequestDto);
		System.out.println("로그아웃 처리할 토큰: " + accessToken);

		logout(accessToken);
	}

	@Transactional
	public void logout(String accessToken) {
		// 헤더 검증
		if (!StringUtils.hasText(accessToken)) {
			System.out.println("토큰이 비어있음");
			throw new JwtException(ErrorCode.INVALID_TOKEN);
		}

		System.out.println("로그아웃 처리 토큰: " + accessToken);

		// 토큰 유효성 검증
		if (!jwtTokenProvider.isTokenValid(accessToken)) {
			throw new JwtException(ErrorCode.INVALID_TOKEN);
		}

		// 토큰에서 이메일 추출
		String email = jwtTokenProvider.getEmailFromToken(accessToken);
		System.out.println("추출된 이메일: " + email);

		// 토큰의 남은 유효 시간 계산
		long remainingTime = jwtTokenProvider.getRemainingTimeFromToken(accessToken);
		System.out.println("토큰 남은 시간: " + remainingTime + "초");

		// 블랙리스트에 토큰 추가
		jwtRedisService.addToBlacklist(accessToken, remainingTime);
		System.out.println("블랙리스트에 토큰 추가 완료");

		// 리프레시 토큰 삭제
		String refreshToken = jwtRedisService.getRefreshToken(email);
		if (refreshToken != null) {
			System.out.println("삭제할 리프레시 토큰: " + refreshToken);
			jwtRedisService.deleteRefreshToken(email);
			System.out.println("리프레시 토큰 삭제 완료");
		} else {
			System.out.println("삭제할 리프레시 토큰이 없습니다.");
		}
	}
}