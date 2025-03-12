package com.ssafy.mmmr.global.config;

import io.swagger.v3.oas.models.ExternalDocumentation;
import io.swagger.v3.oas.models.OpenAPI;
import io.swagger.v3.oas.models.info.Contact;
import io.swagger.v3.oas.models.info.Info;
import io.swagger.v3.oas.models.info.License;
import io.swagger.v3.oas.models.servers.Server;

import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;

import java.util.List;

@Configuration
public class SwaggerConfig {

	// http://localhost:8088/api/swagger-ui/index.html#/

	@Bean
	public OpenAPI customOpenAPI() {
		return new OpenAPI()
			.info(new Info()
				.title("MMMR API Documentation")
				.description("<h3>MMMR Reference for Developers</h3>Swagger를 이용한 MMMR API")
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
				new Server().url("/api").description("Local server")
			));
	}

}


