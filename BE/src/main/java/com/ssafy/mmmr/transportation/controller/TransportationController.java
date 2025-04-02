package com.ssafy.mmmr.transportation.controller;

import java.util.List;

import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.DeleteMapping;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import com.ssafy.mmmr.transportation.dto.MetroRequestDto;
import com.ssafy.mmmr.transportation.dto.TransportationResponseDto;
import com.ssafy.mmmr.transportation.entity.MetroEntity;
import com.ssafy.mmmr.transportation.service.TransportationService;

import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.tags.Tag;
import lombok.RequiredArgsConstructor;

@RestController
@RequestMapping("/api/transportation")
@RequiredArgsConstructor
@Tag(name = "Transportation", description = "교통 정보 API")
public class TransportationController {

	private final TransportationService transportationService;

	@PostMapping("/add")
	public ResponseEntity<MetroEntity> addMetro(@RequestBody MetroRequestDto requestDto) {
		MetroEntity savedMetro = transportationService.addMetro(requestDto);
		return ResponseEntity.ok(savedMetro);
	}
}