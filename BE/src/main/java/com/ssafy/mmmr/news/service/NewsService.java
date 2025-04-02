package com.ssafy.mmmr.news.service;

import com.ssafy.mmmr.global.error.code.ErrorCode;
import com.ssafy.mmmr.global.error.exception.NewsException;
import com.ssafy.mmmr.news.dto.NewsResponseDto;
import com.ssafy.mmmr.news.entity.NewsEntity;
import com.ssafy.mmmr.news.repository.NewsRepository;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.jsoup.Jsoup;
import org.jsoup.nodes.Document;
import org.jsoup.nodes.Element;
import org.jsoup.select.Elements;
import org.springframework.stereotype.Service;

import javax.swing.text.html.HTML;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

@Slf4j
@Service
@RequiredArgsConstructor
public class NewsService {

    private final NewsRepository newsRepository;

    public List<NewsResponseDto> getNewsList() {
        List<NewsEntity> newsEntities = newsRepository.findAll();
        if (newsEntities.isEmpty()) {
            throw new NewsException(ErrorCode.NEWS_FETCH_FAILED);
        }
        List<NewsResponseDto> list = newsEntities.stream()
                .map(newsEntity -> new NewsResponseDto(
                        newsEntity.getId(),
                        newsEntity.getTitle(),
                        newsEntity.getContent()
                ))
                .collect(Collectors.toList());
        return list;
    }

    public void newsCrawler() {
        try {
            String pageUrl = "https://media.naver.com/press/001";

            Document document = Jsoup.connect(pageUrl).timeout(5000).get();
            // 메인 기사 : 총 5개
            Elements elements = document.select(".press_main_news_inner");
            Elements mainArticles = elements.select(".press_news_item");

            for (int i = 0; i < 5; i++) {
                Element article = mainArticles.get(i);
                //개별 기사 제목
                String title = article.select("strong").text().trim();

                // 기사 url
                String articleUrl = article.selectFirst(".press_news_link").attr("href");

                // 기사 내용 중 앞 3문장 반환
                String content = getNewsContent(articleUrl);

                //DB에 저장하는 로직
                NewsEntity newsEntity = NewsEntity.builder()
                        .id((long) i + 1)
                        .title(title)
                        .content(content)
                        .build();
                newsRepository.save(newsEntity);
                log.info("뉴스 크롤링 완료");
            }
        } catch (Exception e) {
            log.error(e.getMessage());
            throw new NewsException(ErrorCode.NEWS_CRAWLING_FAILED);
        }
    }

    public String getNewsContent(String articleUrl) throws IOException {
        // 기사 내용 Element
        Document contentDocument = Jsoup.connect(articleUrl).timeout(5000).get();
        Elements tmp = contentDocument.select("._article_content");

        // 필요 없는 태그&내용 삭제
        tmp.select("img, div, em, strong, span").remove();
        String tmp1 = tmp.html();

        // <br>태그를 기준으로 문단 나누어 리스트로 저장
        List<String> tmpList = new java.util.ArrayList<>(Arrays.stream(tmp1.split("<br>")).toList());

        // 빈 요소 제거
        tmpList.removeIf(tmp2 -> tmp2 == null || tmp2.trim().isEmpty());

        StringBuilder sb = new StringBuilder();

        // 문단이 3개 이하일 경우
        if (tmpList.size() < 3) {
            for (String s : tmpList) {
                sb.append(s);
            }
        } else {
            //문단이 3개 이상일 경우
            for (int i = 0; i < 3; i++) {
                sb.append(tmpList.get(i));
            }
        }
        System.out.println("sb.toString() : " + sb.toString());
        return sb.toString();
    }

}
