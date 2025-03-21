package com.ssafy.mmmr.global.config;

import java.util.List;

import org.springframework.context.annotation.Configuration;
import org.springframework.web.method.support.HandlerMethodArgumentResolver;
import org.springframework.web.servlet.config.annotation.WebMvcConfigurer;

import com.ssafy.mmmr.global.resolver.CurrentUserArgumentResolver;

import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;

@Configuration
@RequiredArgsConstructor
@Slf4j
public class WebConfig implements WebMvcConfigurer {

	private final CurrentUserArgumentResolver currentUserArgumentResolver;

	@Override
	public void addArgumentResolvers(List<HandlerMethodArgumentResolver> resolvers) {
		log.info("CurrentUserArgumentResolver 등록 중...");
		resolvers.add(currentUserArgumentResolver);
		log.info("CurrentUserArgumentResolver 등록 완료");

		// 등록된 리졸버 목록 확인
		for (HandlerMethodArgumentResolver resolver : resolvers) {
			log.info("등록된 ArgumentResolver: {}", resolver.getClass().getName());
		}
	}
}