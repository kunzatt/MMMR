package com.ssafy.mmmr.global.config;

import java.util.Arrays;
import java.util.List;

import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.http.HttpMethod;
import org.springframework.security.config.annotation.method.configuration.EnableMethodSecurity;
import org.springframework.security.config.annotation.web.builders.HttpSecurity;
import org.springframework.security.config.annotation.web.configuration.EnableWebSecurity;
import org.springframework.security.config.http.SessionCreationPolicy;
import org.springframework.security.crypto.bcrypt.BCryptPasswordEncoder;
import org.springframework.security.crypto.password.PasswordEncoder;
import org.springframework.security.web.SecurityFilterChain;
import org.springframework.security.web.authentication.UsernamePasswordAuthenticationFilter;
import org.springframework.web.cors.CorsConfiguration;
import org.springframework.web.cors.CorsConfigurationSource;
import org.springframework.web.cors.UrlBasedCorsConfigurationSource;

import com.ssafy.mmmr.jwt.filter.JwtFilter;

import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;

@Configuration
@EnableWebSecurity
@EnableMethodSecurity
@RequiredArgsConstructor
@Slf4j
public class SecurityConfig {

	private final JwtFilter jwtFilter;

	@Bean
	public SecurityFilterChain securityFilterChain(HttpSecurity http) throws Exception {
		log.info("보안 필터 체인 구성 시작");
		return http
			.csrf(csrf -> csrf.disable())
			.cors(cors -> cors.configurationSource(corsConfigurationSource()))
			.sessionManagement(session -> session.sessionCreationPolicy(SessionCreationPolicy.STATELESS))
			.authorizeHttpRequests(authorize -> authorize
				.requestMatchers(
					// Swagger UI 및 API Docs
					"/v3/api-docs/**",
					"/swagger-ui/**",
					"/swagger-ui.html",
					"/swagger-resources/**",
					"/webjars/**",
					"/api-docs/**",
					"/api/swagger-ui/**",
					"/api/api-docs/**",
					// 인증 및 계정 관련 공개 API
					"/api/accounts",
					"/api/accounts/email-exists",
					"/api/auth/login",
					"api/news",
					"api/mail/**",
					"api/weather"
				)
				.permitAll()
				.anyRequest().authenticated()
			)
			.addFilterBefore(jwtFilter, UsernamePasswordAuthenticationFilter.class)
			.build();
	}

	@Bean
	public CorsConfigurationSource corsConfigurationSource() {
		log.info("CORS 설정 구성 시작");
		CorsConfiguration configuration = new CorsConfiguration();

		// 허용할 오리진 설정
		configuration.setAllowedOriginPatterns(List.of("*"));

		// 모든 HTTP 메서드 명시적으로 허용 (PATCH 메서드 포함)
		configuration.setAllowedMethods(Arrays.asList(
			HttpMethod.GET.name(),
			HttpMethod.POST.name(),
			HttpMethod.PUT.name(),
			HttpMethod.PATCH.name(),
			HttpMethod.DELETE.name(),
			HttpMethod.OPTIONS.name(),
			HttpMethod.HEAD.name()
		));

		// 모든 헤더 허용
		configuration.addAllowedHeader("*");

		// 노출할 헤더 설정
		configuration.setExposedHeaders(Arrays.asList(
			"Authorization",
			"Content-Type",
			"Set-Cookie",
			"Access-Control-Allow-Origin",
			"Access-Control-Allow-Methods",
			"Access-Control-Allow-Headers",
			"Access-Control-Allow-Credentials"
		));

		// preflight 요청 캐시 시간 설정 (3600초 = 1시간)
		configuration.setMaxAge(3600L);

		// 인증 정보 포함 여부 설정
		configuration.setAllowCredentials(true);

		UrlBasedCorsConfigurationSource source = new UrlBasedCorsConfigurationSource();
		source.registerCorsConfiguration("/**", configuration);
		log.info("CORS 설정 완료");
		return source;
	}

	@Bean
	public PasswordEncoder passwordEncoder() {
		return new BCryptPasswordEncoder();
	}
}