package com.ssafy.mmmr.transportation.service;

import org.springframework.stereotype.Service;
import org.springframework.web.client.RestTemplate;

import com.ssafy.mmmr.profiles.repository.ProfileRepository;
import com.ssafy.mmmr.transportation.repository.BusRepository;
import com.ssafy.mmmr.transportation.repository.MetroRepository;

import lombok.RequiredArgsConstructor;

@Service
@RequiredArgsConstructor
public class TransportationService {

	private final MetroRepository metroRepository;
	private final BusRepository busRepository;
	private final ProfileRepository profileRepository;
	private final RestTemplate restTemplate;

}
