package com.ssafy.mmmr.jwt.service;

import java.util.concurrent.TimeUnit;

import org.springframework.data.redis.core.RedisTemplate;
import org.springframework.stereotype.Service;

import lombok.RequiredArgsConstructor;

@Service
@RequiredArgsConstructor
public class JwtRedisService {

	private final RedisTemplate<String, String> redisTemplate;

	// Refresh Token redis에 저장
	public void saveRefreshToken(String email, String refreshToken, long expirartion) {
		redisTemplate.opsForValue().set("Refresh Token : " + email, refreshToken, expirartion, TimeUnit.SECONDS);
	}

	// Refresh Token 조회
	public String getRefreshToken(String email) {
		Object Token = redisTemplate.opsForValue().get("Refresh Token : " + email);
		return Token != null ? Token.toString() : null;
	}

	// Refresh Token 삭제
	public void deleteRefreshToken(String email) {
		try {
			String key = "Refresh Token : " + email;
			System.out.println("Deleting Redis key: " + key);
			redisTemplate.delete(key);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	// 액세스 토큰 블랙리스트에 추가 (로그아웃 등의 경우)
	public void addToBlacklist(String token, long expiration) {
		redisTemplate.opsForValue().set("Black List :" + token, "blacklisted", expiration, TimeUnit.SECONDS);
	}

	// 토큰이 블랙리스트에 있는지 확인
	public boolean isTokenBlacklisted(String token) {
		return Boolean.TRUE.equals(redisTemplate.hasKey("Black List :" + token));
	}

}
