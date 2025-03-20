package com.ssafy.mmmr.jwt.provider;

import com.ssafy.mmmr.global.error.code.ErrorCode;
import com.ssafy.mmmr.global.error.exception.JwtException;
import io.jsonwebtoken.*;
import io.jsonwebtoken.security.Keys;
import jakarta.security.auth.message.AuthException;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Component;

import java.security.Key;
import java.util.Date;
import java.util.function.Function;

@Component
public class JwtTokenProvider {

	@Value("${jwt.secret}")
	private String jwtSecret;

	@Value("${jwt.accessTokenExpiration}")
	private long accessTokenExpiration;

	@Value("${jwt.refreshTokenExpiration}")
	private long refreshTokenExpiration;

	public String generateAccessToken(String email) {
		return generateToken(email, accessTokenExpiration);
	}

	public String generateRefreshToken(String email) {
		return generateToken(email, refreshTokenExpiration);
	}

	private String generateToken(String email, long expiration) {
		Date now = new Date();
		Date expiryDate = new Date(now.getTime() + expiration * 1000);

		return Jwts.builder()
			.setSubject(email)
			.setIssuedAt(now)
			.setExpiration(expiryDate)
			.signWith(getSigningKey(), SignatureAlgorithm.HS512)
			.compact();
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
			Jwts.parser().setSigningKey(getSigningKey()).build().parseClaimsJws(token);
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
			Jwts.parser().setSigningKey(getSigningKey()).build().parseClaimsJws(token);
			return true;
		} catch (Exception e) {
			return false;
		}
	}
}