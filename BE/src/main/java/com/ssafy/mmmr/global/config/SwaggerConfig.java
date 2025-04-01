package com.ssafy.mmmr.global.config;

import io.swagger.v3.oas.models.Components;
import io.swagger.v3.oas.models.ExternalDocumentation;
import io.swagger.v3.oas.models.OpenAPI;
import io.swagger.v3.oas.models.info.Contact;
import io.swagger.v3.oas.models.info.Info;
import io.swagger.v3.oas.models.info.License;
import io.swagger.v3.oas.models.security.SecurityRequirement;
import io.swagger.v3.oas.models.security.SecurityScheme;
import io.swagger.v3.oas.models.servers.Server;

import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;

import java.util.List;

@Configuration
public class SwaggerConfig {

	// http://localhost:8088/api/swagger-ui/index.html#/

	@Bean
	public OpenAPI customOpenAPI() {
		// Security 스키마 설정 - Bearer Authentication
		SecurityScheme bearerScheme = new SecurityScheme()
			.type(SecurityScheme.Type.HTTP)
			.scheme("bearer")
			.bearerFormat("JWT")
			.in(SecurityScheme.In.HEADER)
			.name("Authorization")
			.description("JWT Bearer 토큰을 입력하세요");

		// Security Requirement 객체 생성
		SecurityRequirement securityRequirement = new SecurityRequirement().addList("bearerAuth");

		return new OpenAPI()
			.info(new Info()
				.title("MMMR API Documentation")
				.description("<h3>MMMR Reference for Developers</h3>Swagger를 이용한 MMMR API<br>" +
					"<b>인증 방법:</b> Authorize 버튼을 클릭하여 JWT 토큰을 입력하세요. Bearer 접두사는 자동으로 추가됩니다.")
				.version("v1.0")
				.contact(new Contact()
					.name("Support Team")
					.email("support@mmmr.com")
					.url("https://lab.ssafy.com/s12-mobility-smarthome-sub1/S12P21A703"))
				.license(new License()
					.name("MMMR License")
					.url("https://lab.ssafy.com/s12-mobility-smarthome-sub1/S12P21A703")))
			.externalDocs(new ExternalDocumentation()
				.description("MMMR Documentation")
				.url("https://lab.ssafy.com/s12-mobility-smarthome-sub1/S12P21A703"))
			.servers(List.of(
				new Server().url("").description("Local server")
			))
			.components(new Components()
				.addSecuritySchemes("bearerAuth", bearerScheme))
			.addSecurityItem(securityRequirement);
	}
}