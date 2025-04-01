package com.ssafy.mmmr.videos.service;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.ssafy.mmmr.global.error.code.ErrorCode;
import com.ssafy.mmmr.global.error.exception.VideoException;
import com.ssafy.mmmr.videos.client.VideoClient;
import com.ssafy.mmmr.videos.dto.VideoResponseDto;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Service;

import java.util.List;
import java.util.Objects;
import java.util.stream.StreamSupport;

@Slf4j
@Service
@RequiredArgsConstructor
public class VideoService {

    private final ObjectMapper objectMapper;
    private final VideoClient videoClient;

    public List<VideoResponseDto> searchVideoIdByKeyword(String keyword){
        try{

            String videoInfo = videoClient.searchVideoIdByKeyword(keyword);
            JsonNode videoInfoJson = objectMapper.readTree(videoInfo);

            // videoId를 추출하여 리스트로 전환
            // 1. json 객체에서 stream을 사용할 수 있도록 변환
            List<VideoResponseDto> result = StreamSupport.stream(videoInfoJson.get("items").spliterator(), false)
                    // 2. item 배열 안의 id 객체 안의 videoId 객체를 찾음
                    .map(item -> item.path("id").path("videoId").asText())
                    .map(VideoResponseDto::new)
                    .toList();

            if(result.isEmpty()){
                log.error("동영상 조회 결과가 없음");
               throw new VideoException(ErrorCode.VIDEO_NOT_FOUND);
            }
            return result;

        }catch(Exception e){
            log.error("동영상 정보 조회 중 오류 발생");
            throw new VideoException(ErrorCode.VIDEO_FETCH_FAILED);
        }

    }

}
