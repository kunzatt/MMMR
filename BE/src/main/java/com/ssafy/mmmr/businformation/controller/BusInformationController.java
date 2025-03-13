package com.ssafy.mmmr.businformation.controller;

import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;
import org.springframework.web.multipart.MultipartFile;

import com.ssafy.mmmr.businformation.service.BusInformationService;
import com.ssafy.mmmr.global.error.dto.ErrorResponse;

import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.Parameter;
import io.swagger.v3.oas.annotations.media.Content;
import io.swagger.v3.oas.annotations.media.ExampleObject;
import io.swagger.v3.oas.annotations.media.Schema;
import io.swagger.v3.oas.annotations.responses.ApiResponse;
import io.swagger.v3.oas.annotations.responses.ApiResponses;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;

@RestController
@RequestMapping("/api/bus-information")
@RequiredArgsConstructor
@Slf4j
public class BusInformationController {

	private final BusInformationService busInformationService;

	@Operation(summary = "버스 정보 CSV 업로드", description = "버스 정보가 담긴 CSV 파일을 업로드하고 Spring Batch로 처리합니다.")
	@ApiResponses({
		@ApiResponse(
			responseCode = "200",
			description = "파일 업로드 및 처리 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(type = "string"),
				examples = @ExampleObject(value = "\"파일 업로드 및 DB 저장 JobId: 12345\"")
			)
		),
		@ApiResponse(
			responseCode = "400",
			description = "잘못된 파일 형식",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = {
					@ExampleObject(
						name = "CSV 파일이 아님",
						value = """
                        {
                            "timestamp": "2024-01-23T10:00:00",
                            "status": 400,
                            "message": "Only CSV files are supported",
                            "errors": []
                        }
                        """
					)
				}
			)
		),
		@ApiResponse(
			responseCode = "500",
			description = "서버 오류",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = {
					@ExampleObject(
						name = "파일 처리 오류",
						value = """
                        {
                            "timestamp": "2024-01-23T10:00:00",
                            "status": 500,
                            "message": "파일 업로드 중 오류가 발생했습니다",
                            "errors": []
                        }
                        """
					)
				}
			)
		)
	})
	@PostMapping(value = "/upload", consumes = MediaType.MULTIPART_FORM_DATA_VALUE)
	public ResponseEntity<String> uploadBusInformationFile(
		@Parameter(
			description = "업로드할 버스 정보 CSV 파일",
			required = true,
			content = @Content(mediaType = MediaType.MULTIPART_FORM_DATA_VALUE),
			schema = @Schema(type = "string", format = "binary")
		)
		@RequestParam("file") MultipartFile file
	) {
		log.info("버스 정보 CSV 파일 업로드: {}", file.getOriginalFilename());
		String result = busInformationService.uploadAndProcessFile(file);
		return ResponseEntity.ok(result);
	}
}
