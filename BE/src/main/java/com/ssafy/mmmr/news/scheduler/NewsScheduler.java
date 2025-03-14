package com.ssafy.mmmr.news.scheduler;

import com.ssafy.mmmr.news.entity.NewsEntity;
import com.ssafy.mmmr.news.repository.NewsRepository;
import org.jsoup.Jsoup;
import org.jsoup.nodes.Document;
import org.jsoup.nodes.Element;
import org.jsoup.select.Elements;
import org.springframework.scheduling.annotation.Scheduled;
import org.springframework.stereotype.Component;

import java.io.IOException;
import java.util.Objects;

@Component
public class NewsScheduler {

    private final NewsRepository newsRepository;

    public NewsScheduler(NewsRepository newsRepository) {
        this.newsRepository = newsRepository;
    }

    @Scheduled(cron = "0 0 0/1 * * *")
    public void newsScheduler() throws IOException {
        String pageUrl = "https://www.yna.co.kr/";

        Document document = Jsoup.connect(pageUrl).timeout(5000).get();
        // 메인 기사 : 총 5개
        Elements elements = document.select(".top-main-news001");
        Elements mainArticles = elements.select(".news-con");

        for (Element article : mainArticles) {
            //개별 기사 제목
            String title = article.select(".title01").text().trim();
            // 기사 링크
            String articleUrl = Objects.requireNonNull(article.select(
                                    ".tit-con:nth-of-type("
                                            + (mainArticles.indexOf(article) + 1)
                                            + ")")
                                    .first())
                                    .attr("href");

            Document contentDocument = Jsoup.connect(articleUrl).timeout(5000).get();
            // 기사 내용
            String content = contentDocument.select(".story-news.article").text();
            //DB에 저장하는 로직
            NewsEntity newsEntity = NewsEntity.builder()
                    .title(title)
                    .content(content)
                    .build();
            newsRepository.create(newsEntity);

        }


    }
}
