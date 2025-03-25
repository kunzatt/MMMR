package com.ssafy.mmmr.jwt.provider;

import com.ssafy.mmmr.global.error.code.ErrorCode;
import com.ssafy.mmmr.global.error.exception.JwtException;
import io.jsonwebtoken.*;
import io.jsonwebtoken.security.Keys;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Component;

import java.security.Key;
import java.util.Date;
import java.util.function.Function;
import java.util.regex.Pattern;

import lombok.extern.slf4j.Slf4j;

@Component
public class JwtTokenProvider {

	@Value("${jwt.secret}")
	private String jwtSecret;

	@Value("${jwt.accessTokenExpiration}")
	private long accessTokenExpiration;

	@Value("${jwt.refreshTokenExpiration}")
	private long refreshTokenExpiration;

	// JWT 토큰 형식을 확인하는 정규 표현식 패턴
	private static final Pattern JWT_PATTERN = Pattern.compile("^[A-Za-z0-9-_=]+\\.[A-Za-z0-9-_=]+\\.[A-Za-z0-9-_.+/=]*$");

	public String generateAccessToken(String email) {
		return generateToken(email, accessTokenExpiration);
	}

	public String generateRefreshToken(String email) {
		return generateToken(email, refreshTokenExpiration);
	}

	private String generateToken(String email, long expiration) {
		Date now = new Date();
		Date expiryDate = new Date(now.getTime() + expiration * 1000);

		String token = Jwts.builder()
			.setSubject(email)
			.setIssuedAt(now)
			.setExpiration(expiryDate)
			.signWith(getSigningKey(), SignatureAlgorithm.HS512)
			.compact();
		return token;
	}

	private Key getSigningKey() {
		byte[] keyBytes = jwtSecret.getBytes();
		return Keys.hmacShaKeyFor(keyBytes);
	}

	public String getEmailFromToken(String token) {
		return getClaimFromToken(token, Claims::getSubject);
	}

	public Date getExpirationDateFromToken(String token) {
		return getClaimFromToken(token, Claims::getExpiration);
	}

	public <T> T getClaimFromToken(String token, Function<Claims, T> claimsResolver) {
		final Claims claims = getAllClaimsFromToken(token);
		return claimsResolver.apply(claims);
	}

	private Claims getAllClaimsFromToken(String token) {
		try {
			// 토큰 형식 기본 검증
			if (!isValidJwtFormat(token)) {
				throw new JwtException(ErrorCode.INVALID_TOKEN);
			}

			return Jwts.parser()
				.setSigningKey(getSigningKey())
				.build()
				.parseClaimsJws(token)
				.getBody();
		} catch (ExpiredJwtException e) {
			throw new JwtException(ErrorCode.EXPIRED_TOKEN);
		} catch (JwtException | IllegalArgumentException e) {
			throw new JwtException(ErrorCode.INVALID_TOKEN);
		}
	}

	public boolean validateToken(String token) {
		try {
			// 토큰 형식 기본 검증
			if (!isValidJwtFormat(token)) {
				return false;
			}

			Jwts.parser()
				.setSigningKey(getSigningKey())
				.build()
				.parseClaimsJws(token);
			return true;
		} catch (ExpiredJwtException e) {
			throw new JwtException(ErrorCode.EXPIRED_TOKEN);
		} catch (JwtException | IllegalArgumentException e) {
			throw new JwtException(ErrorCode.INVALID_TOKEN);
		}
	}

	public long getRemainingTimeFromToken(String token) {
		try {
			Date expiration = getExpirationDateFromToken(token);
			return (expiration.getTime() - new Date().getTime()) / 1000;
		} catch (Exception e) {
			return 0;
		}
	}

	public boolean isTokenValid(String token) {
		if (token == null) {
			return false;
		}

		try {
			Jwts.parser()
				.setSigningKey(getSigningKey())
				.build()
				.parseClaimsJws(token);
			return true;
		} catch (Exception e) {
			return false;
		}
	}

	private boolean isValidJwtFormat(String token) {
		if (token == null || token.isEmpty()) {
			return false;
		}

		// 기본 형식 확인: 점(.)으로 구분된 세 부분으로 구성되어 있어야 함
		String[] parts = token.split("\\.");
		if (parts.length != 3) {
			return false;
		}

		// 토큰은 'e'로 시작해야 함 (ey...)
		if (!token.startsWith("e")) {
			return false;
		}

		// JWT 패턴 확인
		boolean matches = JWT_PATTERN.matcher(token).matches();
		if (!matches) {
		}

		return matches;
	}
}