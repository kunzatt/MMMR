package com.ssafy.mmmr.videos.service;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.ssafy.mmmr.global.error.code.ErrorCode;
import com.ssafy.mmmr.global.error.exception.VideoException;
import com.ssafy.mmmr.videos.client.VideoClient;
import com.ssafy.mmmr.videos.dto.VideoResponseDto;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.hibernate.validator.constraints.Range;
import org.springframework.stereotype.Service;

import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.stream.StreamSupport;

@Slf4j
@Service
@RequiredArgsConstructor
public class VideoService {

    private final ObjectMapper objectMapper;
    private final VideoClient videoClient;

    public List<VideoResponseDto> searchVideoIdByKeyword(String keyword) throws JsonProcessingException {

        String videoInfo = videoClient.searchVideoIdByKeyword(keyword);
        JsonNode videoInfoJson = objectMapper.readTree(videoInfo);

        // 1. json 객체에서 stream을 사용할 수 있도록 변환
        List<VideoResponseDto> videoResponseDtoList = StreamSupport.stream(videoInfoJson.get("items").spliterator(), false)
                // 2. item 배열 안의 id 객체 안의 videoId 객체를 찾음
                .map(item -> item.path("id").path("videoId").asText(null))
                .filter(Objects::nonNull)
                .map(VideoResponseDto::new)
                .toList();

        List<VideoResponseDto> result = Optional.of(videoResponseDtoList)
                .filter(list -> !list.isEmpty())
                .orElseThrow(() -> {
                    log.error("동영상 조회 결과가 없음");
                    return new VideoException(ErrorCode.VIDEO_NOT_FOUND);
                });
        return result.subList(0, 5);
    }
}
