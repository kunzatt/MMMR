package com.ssafy.mmmr.videos.client;


import com.ssafy.mmmr.global.error.code.ErrorCode;
import com.ssafy.mmmr.global.error.exception.VideoException;
import lombok.RequiredArgsConstructor;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.cache.annotation.Cacheable;
import org.springframework.http.HttpEntity;
import org.springframework.http.HttpHeaders;
import org.springframework.http.HttpMethod;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Component;
import org.springframework.web.client.RestTemplate;
import org.springframework.web.util.UriComponentsBuilder;

import java.net.URI;

@Component
@RequiredArgsConstructor
public class VideoClient {

   private final RestTemplate videoRestTemplate;

   @Value("${video.api.key}")
   private String apiKey;

   @Value("${video.api.url}")
   private String apiUrl;

   @Cacheable(value = "videoCache", key = "#keyword")
   public String searchVideoIdByKeyword(String keyword) {
       URI uri = UriComponentsBuilder.fromHttpUrl(apiUrl)
			.queryParam("key", apiKey)
			.queryParam("q", keyword)
			.queryParam("maxResults", 5)
			.queryParam("type", "video")
			.build()
			.toUri();

        try {
			HttpHeaders headers = new HttpHeaders();
			HttpEntity<String> entity = new HttpEntity<>(headers);

			ResponseEntity<String> response = videoRestTemplate.exchange(uri, HttpMethod.GET, entity, String.class);
            return response.getBody();

		} catch (Exception e) {
			throw new VideoException(ErrorCode.VIDEO_FETCH_FAILED);
		}

   }



}
