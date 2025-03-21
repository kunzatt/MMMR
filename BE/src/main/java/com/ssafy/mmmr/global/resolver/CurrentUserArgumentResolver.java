package com.ssafy.mmmr.global.resolver;

import org.springframework.core.MethodParameter;
import org.springframework.security.authentication.UsernamePasswordAuthenticationToken;
import org.springframework.security.core.Authentication;
import org.springframework.security.core.context.SecurityContextHolder;
import org.springframework.security.core.userdetails.UserDetails;
import org.springframework.security.core.userdetails.UsernameNotFoundException;
import org.springframework.stereotype.Component;
import org.springframework.web.bind.support.WebDataBinderFactory;
import org.springframework.web.context.request.NativeWebRequest;
import org.springframework.web.method.support.HandlerMethodArgumentResolver;
import org.springframework.web.method.support.ModelAndViewContainer;

import com.ssafy.mmmr.account.dto.AuthUser;
import com.ssafy.mmmr.account.entity.AccountEntity;
import com.ssafy.mmmr.account.repository.AccountRepository;
import com.ssafy.mmmr.global.annotation.CurrentUser;

import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;

@Component
@RequiredArgsConstructor
public class CurrentUserArgumentResolver implements HandlerMethodArgumentResolver {

	private final AccountRepository accountRepository;

	@Override
	public boolean supportsParameter(MethodParameter parameter) {
		return parameter.hasParameterAnnotation(CurrentUser.class)
			&& parameter.getParameterType().equals(AuthUser.class);
	}

	@Override
	public Object resolveArgument(MethodParameter parameter, ModelAndViewContainer mavContainer,
		NativeWebRequest webRequest, WebDataBinderFactory binderFactory) {

		try {
			Authentication authentication = SecurityContextHolder.getContext().getAuthentication();

			if (authentication == null) {
				return null;
			}

			String email;
			if (authentication.getPrincipal() instanceof UserDetails) {
				UserDetails userDetails = (UserDetails) authentication.getPrincipal();
				email = userDetails.getUsername();
			} else {
				email = authentication.getName();
			}

			if (email == null || email.isEmpty()) {
				return null;
			}

			// 이메일로 계정 조회
			AccountEntity account = accountRepository.findByEmail(email)
				.orElseThrow(() -> {
					return new UsernameNotFoundException("사용자를 찾지 못했습니다: " + email);
				});

			// 조회한 계정 정보로 AuthUser 객체 생성
			AuthUser authUser = AuthUser.from(account);

			return authUser;
		} catch (Exception ex) {
			return null;
		}
	}
}