package com.ssafy.mmmr.weather.dto;

import lombok.Builder;
import lombok.Getter;

@Getter
@Builder
public class WeatherResponseDto {
	private String currentWeather;
	private int currentTemperature;
	private int minTemperature;
	private int maxTemperature;
	private int humidity;
	private int precipitation;
	private String clothingAdvice;
	private String umbrellaAdvice;
}