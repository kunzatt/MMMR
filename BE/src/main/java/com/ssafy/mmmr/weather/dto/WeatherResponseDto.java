package com.ssafy.mmmr.weather.dto;

import lombok.Builder;
import lombok.Getter;

@Getter
@Builder
public class WeatherResponseDto {
	private String currentWeather;
	private double currentTemperature;
	private double minTemperature;
	private double maxTemperature;
	private int humidity;
	private int precipitation;
	private String clothingAdvice;
	private String umbrellaAdvice;
	private boolean hasAlert;
	private String alertMessage;
	private boolean hasTyphoon;
	private String typhoonMessage;
}