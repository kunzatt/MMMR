package com.ssafy.mmmr.jwt.filter;

import java.io.IOException;
import java.util.regex.Pattern;

import org.springframework.security.authentication.UsernamePasswordAuthenticationToken;
import org.springframework.security.core.context.SecurityContextHolder;
import org.springframework.security.core.userdetails.UserDetails;
import org.springframework.security.core.userdetails.UserDetailsService;
import org.springframework.security.web.authentication.WebAuthenticationDetailsSource;
import org.springframework.stereotype.Component;
import org.springframework.util.StringUtils;
import org.springframework.web.filter.OncePerRequestFilter;

import com.ssafy.mmmr.global.error.code.ErrorCode;
import com.ssafy.mmmr.global.error.exception.JwtException;
import com.ssafy.mmmr.jwt.provider.JwtTokenProvider;
import com.ssafy.mmmr.jwt.service.JwtRedisService;

import jakarta.servlet.FilterChain;
import jakarta.servlet.ServletException;
import jakarta.servlet.http.HttpServletRequest;
import jakarta.servlet.http.HttpServletResponse;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;

@Component
@RequiredArgsConstructor
@Slf4j
public class JwtFilter extends OncePerRequestFilter {

	private final JwtTokenProvider jwtTokenProvider;
	private final JwtRedisService jwtRedisService;
	private final UserDetailsService userDetailsService;

	// JWT 토큰 형식을 확인하는 정규 표현식 패턴 (간단한 검증용)
	private static final Pattern JWT_PATTERN = Pattern.compile("^[A-Za-z0-9-_=]+\\.[A-Za-z0-9-_=]+\\.[A-Za-z0-9-_.+/=]*$");

	@Override
	protected void doFilterInternal(HttpServletRequest request, HttpServletResponse response, FilterChain filterChain)
		throws ServletException, IOException {
		try {
			String token = getTokenFromRequest(request);
			log.debug("요청 URI: {}", request.getRequestURI());

			if (StringUtils.hasText(token)) {
				log.debug("토큰 발견: {}", token.substring(0, Math.min(10, token.length())) + "...");

				// 토큰 형식 검증
				if (!isValidJwtFormat(token)) {
					log.error("유효하지 않은 JWT 형식: {}", token.substring(0, Math.min(10, token.length())) + "...");
					SecurityContextHolder.clearContext();
					filterChain.doFilter(request, response);
					return;
				}

				if (jwtTokenProvider.validateToken(token)) {
					if (jwtRedisService.isTokenBlacklisted(token)) {
						log.warn("블랙리스트에 있는 토큰입니다");
						throw new JwtException(ErrorCode.TOKEN_NOT_FOUND);
					}

					String email = jwtTokenProvider.getEmailFromToken(token);
					log.debug("토큰에서 추출한 이메일: {}", email);

					UserDetails userDetails = userDetailsService.loadUserByUsername(email);
					UsernamePasswordAuthenticationToken authentication = new UsernamePasswordAuthenticationToken(
						userDetails, null, userDetails.getAuthorities());

					authentication.setDetails(new WebAuthenticationDetailsSource().buildDetails(request));
					SecurityContextHolder.getContext().setAuthentication(authentication);

					// 디버깅을 위한 인증 정보 출력
					log.debug("사용자 인증 성공: {}", email);
					log.debug("인증 principal: {}", authentication.getPrincipal());
					log.debug("인증 클래스: {}", authentication.getClass().getName());
					log.debug("SecurityContext 인증 객체: {}", SecurityContextHolder.getContext().getAuthentication());
				} else {
					log.warn("유효하지 않은 토큰");
				}
			}
		} catch (Exception e) {
			log.error("JWT 인증 실패: {}", e.getMessage());
			SecurityContextHolder.clearContext();
		}

		filterChain.doFilter(request, response);
	}

	private String getTokenFromRequest(HttpServletRequest request) {
		String bearerToken = request.getHeader("Authorization");
		log.debug("Authorization 헤더: {}", bearerToken);

		// 토큰이 없는 경우 처리
		if (!StringUtils.hasText(bearerToken)) {
			return null;
		}

		// Bearer 접두사가 있는지 확인하고 처리
		if (bearerToken.startsWith("Bearer ")) {
			return bearerToken.substring(7);
		}

		// Bearer 접두사가 없는 경우 그대로 반환
		return bearerToken;
	}

	/**
	 * JWT 토큰 형식이 유효한지 확인
	 * 토큰은 반드시 세 부분으로 나누어져 있어야 하며, 각 부분은 점(.)으로 구분됨
	 * 각 부분은 Base64URL로 인코딩된 문자열이어야 함
	 * @param token 검사할 JWT 토큰
	 * @return 형식이 유효하면 true, 그렇지 않으면 false
	 */
	private boolean isValidJwtFormat(String token) {
		if (token == null || token.isEmpty()) {
			return false;
		}

		// 정규 표현식 패턴 매칭
		boolean isValidFormat = JWT_PATTERN.matcher(token).matches();

		// 추가 검증: JWT 토큰은 반드시 "e"로 시작해야 함
		boolean startsWithCorrectChar = token.startsWith("e");

		if (!isValidFormat) {
			log.warn("JWT 토큰 형식이 유효하지 않습니다");
		}

		if (!startsWithCorrectChar) {
			log.warn("JWT 토큰이 'e'로 시작하지 않습니다");
		}

		return isValidFormat && startsWithCorrectChar;
	}
}